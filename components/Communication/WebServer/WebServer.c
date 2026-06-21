#include "WebServer.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "cJSON.h"

static const char *TAG = "WebServer";

#define MAX_WS_CLIENTS  4
#define DNS_PORT        53

static httpd_handle_t s_http_server = NULL;
static web_command_callback_t s_command_callback = NULL;
static int ws_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static TaskHandle_t s_dns_task = NULL;

static const char *index_html =
"<!DOCTYPE html>\n"
"<html lang=\"zh-CN\">\n"
"<head>\n"
"    <meta charset=\"UTF-8\">\n"
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\n"
"    <title>机械臂控制器</title>\n"
"    <style>\n"
"        * { margin:0; padding:0; box-sizing:border-box; }\n"
"        body { font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif; background:linear-gradient(135deg,#1a1a2e 0%,#16213e 100%); min-height:100vh; color:#fff; }\n"
"        .container { max-width:860px; margin:0 auto; padding:20px; }\n"
"        h1 { text-align:center; margin-bottom:30px; color:#00d9ff; text-shadow:0 0 10px rgba(0,217,255,0.5); font-size:1.6em; }\n"
"        .section { background:rgba(255,255,255,0.05); border-radius:15px; padding:20px; margin-bottom:20px; }\n"
"        .section h2 { color:#00d9ff; margin-bottom:15px; font-size:1.1em; display:flex; align-items:center; gap:10px; }\n"
"        .control-panel { display:grid; grid-template-columns:repeat(3, 1fr); gap:12px; }\n"
"        @media(max-width:600px){ .control-panel { grid-template-columns:repeat(2, 1fr); } }\n"
"        .servo-control { background:rgba(0,0,0,0.3); border-radius:10px; padding:12px; }\n"
"        .servo-control label { display:block; margin-bottom:3px; font-size:0.85em; font-weight:bold; color:#ccc; }\n"
"        .servo-control input[type=\"range\"] { width:100%; height:6px; border-radius:3px; background:#333; outline:none; -webkit-appearance:none; }\n"
"        .servo-control input[type=\"range\"]::-webkit-slider-thumb { -webkit-appearance:none; width:22px; height:22px; border-radius:50%; background:#00d9ff; cursor:pointer; box-shadow:0 0 10px rgba(0,217,255,0.5); }\n"
"        .servo-control .val-row { display:flex; justify-content:space-between; align-items:center; margin-top:5px; }\n"
"        .servo-control .val-row span { background:#00d9ff; color:#000; padding:3px 8px; border-radius:5px; font-weight:bold; font-size:0.85em; }\n"
"        .servo-control .val-row input { width:55px; background:rgba(255,255,255,0.1); border:1px solid #555; color:#fff; padding:3px 5px; border-radius:5px; text-align:center; font-size:0.85em; }\n"
"        .status-grid { display:grid; grid-template-columns:repeat(4, 1fr); gap:10px; }\n"
"        @media(max-width:600px){ .status-grid { grid-template-columns:repeat(2, 1fr); } }\n"
"        .status-item { background:rgba(0,0,0,0.3); border-radius:8px; padding:10px; text-align:center; }\n"
"        .status-item .label { font-size:0.75em; color:#888; }\n"
"        .status-item .value { font-size:1.1em; font-weight:bold; color:#00ff88; }\n"
"        .btn { background:linear-gradient(45deg,#00d9ff,#00ff88); color:#000; border:none; padding:10px 20px; border-radius:8px; font-weight:bold; cursor:pointer; transition:all 0.3s; font-size:0.9em; }\n"
"        .btn:hover { transform:scale(1.05); box-shadow:0 0 20px rgba(0,217,255,0.5); }\n"
"        .btn-group { display:flex; gap:10px; justify-content:center; flex-wrap:wrap; margin-top:12px; }\n"
"        .conn-dot { display:inline-block; width:10px; height:10px; border-radius:50%; }\n"
"        .conn-dot.on { background:#00ff88; box-shadow:0 0 10px #00ff88; animation:pulse 2s infinite; }\n"
"        .conn-dot.off { background:#ff4444; }\n"
"        @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.5} }\n"
"        .mode-tag { padding:4px 12px; border-radius:20px; font-weight:bold; font-size:0.8em; }\n"
"        .mode-imu { background:#00ff88; color:#000; }\n"
"        .mode-manual { background:#ff8800; color:#000; }\n"
"    </style>\n"
"</head>\n"
"<body>\n"
"    <div id=\"firefox-hint\" style=\"display:none;background:#ff8800;color:#000;text-align:center;padding:10px 15px;font-size:0.9em;font-weight:bold;\">\n"
"        传感器不可用？复制 <b>192.168.4.1</b> 到 <b>Firefox 浏览器</b> 打开\n"
"    </div>\n"
"    <div class=\"container\">\n"
"        <h1>四自由度机械臂控制器</h1>\n"
"        <div class=\"section\">\n"
"            <h2><span id=\"conn-dot\" class=\"conn-dot off\"></span>实时状态</h2>\n"
"            <div class=\"status-grid\">\n"
"                <div class=\"status-item\"><div class=\"label\">Pitch</div><div class=\"value\" id=\"pitch\">0.0</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">Roll</div><div class=\"value\" id=\"roll\">0.0</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">模式</div><div class=\"value\"><span id=\"mode-tag\" class=\"mode-tag mode-manual\">手动</span></div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">连接数</div><div class=\"value\" id=\"stations\">0</div></div>\n"
"            </div>\n"
"        </div>\n"
"        <div class=\"section\">\n"
"            <h2><span id=\"mode-label\" class=\"mode-tag mode-manual\">手动模式</span> — 舵机控制</h2>\n"
"            <div class=\"control-panel\">\n"
"                <div class=\"servo-control\"><label>底座 (0~180)</label><input type=\"range\" id=\"s0\" min=\"0\" max=\"180\" value=\"90\"><div class=\"val-row\"><span id=\"sv0\">90</span><input id=\"si0\" value=\"90\" onchange=\"onNum(0)\"></div></div>\n"
"                <div class=\"servo-control\"><label>肩 Shoulder (60~120)</label><input type=\"range\" id=\"s1\" min=\"60\" max=\"120\" value=\"90\"><div class=\"val-row\"><span id=\"sv1\">90</span><input id=\"si1\" value=\"90\" onchange=\"onNum(1)\"></div></div>\n"
"                <div class=\"servo-control\"><label>肘 Elbow (40~140)</label><input type=\"range\" id=\"s2\" min=\"40\" max=\"140\" value=\"90\"><div class=\"val-row\"><span id=\"sv2\">90</span><input id=\"si2\" value=\"90\" onchange=\"onNum(2)\"></div></div>\n"
"                <div class=\"servo-control\"><label>腕 Wrist (20~160)</label><input type=\"range\" id=\"s3\" min=\"20\" max=\"160\" value=\"90\"><div class=\"val-row\"><span id=\"sv3\">90</span><input id=\"si3\" value=\"90\" onchange=\"onNum(3)\"></div></div>\n"
"                <div class=\"servo-control\"><label>夹爪 Gripper (0~180)</label><input type=\"range\" id=\"s4\" min=\"0\" max=\"180\" value=\"180\"><div class=\"val-row\"><span id=\"sv4\">180</span><input id=\"si4\" value=\"180\" onchange=\"onNum(4)\"></div></div>\n"
"            </div>\n"
"        <div class=\"section\">\n"
"            <h2>手机姿态</h2>\n"
"            <div class=\"status-grid\">\n"
"                <div class=\"status-item\"><div class=\"label\">手机 Pitch</div><div class=\"value\" id=\"phone-pitch\">--</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">手机 Roll</div><div class=\"value\" id=\"phone-roll\">--</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">传感器</div><div class=\"value\" id=\"imu-perm\">待启动</div></div>\n"
"            </div>\n"
"            <div class=\"btn-group\">\n"
"                <button class=\"btn\" id=\"sensor-btn\" onclick=\"startSensor()\">启动传感器</button>\n"
"            </div>\n"
"        </div>\n"
"            <div class=\"btn-group\">\n"
"                <button class=\"btn\" onclick=\"sendCmd('reset',0)\">归位</button>\n"
"                <button class=\"btn\" onclick=\"sendCmd('wave',0)\">挥手</button>\n"
"                <button class=\"btn\" onclick=\"sendCmd('mode',0)\">手动控制</button>\n"
"                <button class=\"btn\" onclick=\"sendCmd('mode',1)\">MPU6050</button>\n"
"                <button class=\"btn\" onclick=\"sendCmd('mode',2)\">手机 IMU</button>\n"
"            </div>\n"
"        </div>\n"
"        <div class=\"section\">\n"
"            <h2>实际角度</h2>\n"
"            <div class=\"status-grid\">\n"
"                <div class=\"status-item\"><div class=\"label\">底座</div><div class=\"value\" id=\"base\">90</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">肩关节</div><div class=\"value\" id=\"shoulder\">90</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">肘关节</div><div class=\"value\" id=\"elbow\">90</div></div>\n"
"                <div class=\"status-item\"><div class=\"label\">腕关节</div><div class=\"value\" id=\"wrist\">90</div></div>\n"
"            </div>\n"
"        </div>\n"
"    </div>\n"
"    <script>\n"
"        var ws=null, rt=null, phonePitch=0, phoneRoll=0, imuReady=false, imuActive=false, imuType='';\n"
"        function onOrient(e){\n"
"            if(e.beta===null&&e.gamma===null)return;\n"
"            phonePitch=e.beta||0;phoneRoll=e.gamma||0;\n"
"            document.getElementById('phone-pitch').textContent=phonePitch.toFixed(1);\n"
"            document.getElementById('phone-roll').textContent=phoneRoll.toFixed(1);\n"
"        }\n"
"        function onMotion(e){\n"
"            if(imuType)return;\n"
"            var a=e.accelerationIncludingGravity;if(!a)return;\n"
"            phonePitch=Math.atan2(a.x,Math.sqrt(a.y*a.y+a.z*a.z))*180/Math.PI;\n"
"            phoneRoll=Math.atan2(a.y,Math.sqrt(a.x*a.x+a.z*a.z))*180/Math.PI;\n"
"            document.getElementById('phone-pitch').textContent=phonePitch.toFixed(1);\n"
"            document.getElementById('phone-roll').textContent=phoneRoll.toFixed(1);\n"
"        }\n"
"        function startSensor(){\n"
"            if(imuActive)return;\n"
"            imuActive=true;\n"
"            var permEl=document.getElementById('imu-perm');\n"
"            var btn=document.getElementById('sensor-btn');\n"
"            permEl.textContent='检测中...';\n"
"            /* iOS 13+: request permission */\n"
"            if(typeof DeviceOrientationEvent!=='undefined'&&typeof DeviceOrientationEvent.requestPermission==='function'){\n"
"                DeviceOrientationEvent.requestPermission().then(function(s){\n"
"                    if(s==='granted'){\n"
"                        window.addEventListener('deviceorientation',onOrient);\n"
"                        imuReady=true;imuType='orient';\n"
"                        permEl.textContent='OK';btn.textContent='已启动';btn.disabled=true;\n"
"                    }else{permEl.textContent='被拒绝';imuActive=false;}\n"
"                }).catch(function(){permEl.textContent='权限错误';imuActive=false;});\n"
"                return;\n"
"            }\n"
"            /* Android/desktop: register both, use whichever fires */\n"
"            window.addEventListener('deviceorientation',function(e){\n"
"                if(e.beta!==null||e.gamma!==null){\n"
"                    if(!imuReady){imuReady=true;imuType='orient';permEl.textContent='OK';btn.textContent='已启动';btn.disabled=true;}\n"
"                    onOrient(e);\n"
"                }\n"
"            });\n"
"            window.addEventListener('devicemotion',function(e){\n"
"                if(!imuReady&&e.accelerationIncludingGravity&&e.accelerationIncludingGravity.x!==null){\n"
"                    imuReady=true;imuType='accel';permEl.textContent='OK(重力)';btn.textContent='已启动';btn.disabled=true;\n"
"                }\n"
"                if(!imuReady||imuType==='accel')onMotion(e);\n"
"            });\n"
"            setTimeout(function(){\n"
"                if(!imuReady){permEl.textContent='不支持';document.getElementById('firefox-hint').style.display='block';imuActive=false;}\n"
"            },3000);\n"
"        }\n"
"        setInterval(function(){\n"
"            if(wsOk&&imuReady)ws.send(JSON.stringify({command:'phone_imu',pitch:Math.round(phonePitch*10)/10,roll:Math.round(phoneRoll*10)/10}));\n"
"        },100);\n"
"        function conn(){\n"
"            ws=new WebSocket('ws://'+window.location.host+'/ws');\n"
"            ws.onopen=function(){wsOk=true;document.getElementById('conn-dot').className='conn-dot on';};\n"
"            ws.onclose=function(){wsOk=false;document.getElementById('conn-dot').className='conn-dot off';rt=setTimeout(conn,3000);};\n"
"            ws.onmessage=function(e){\n"
"                try{var d=JSON.parse(e.data);updateUI(d);}catch(x){}\n"
"            };\n"
"        }\n"
"        function updateUI(d){\n"
"            if(d.pitch!==undefined)document.getElementById('pitch').textContent=d.pitch.toFixed(1);\n"
"            if(d.roll!==undefined)document.getElementById('roll').textContent=d.roll.toFixed(1);\n"
"            if(d.base!==undefined)document.getElementById('base').textContent=d.base.toFixed(1);\n"
"            if(d.shoulder!==undefined)document.getElementById('shoulder').textContent=d.shoulder.toFixed(1);\n"
"            if(d.elbow!==undefined)document.getElementById('elbow').textContent=d.elbow.toFixed(1);\n"
"            if(d.wrist!==undefined)document.getElementById('wrist').textContent=d.wrist.toFixed(1);\n"
"            if(d.stations!==undefined)document.getElementById('stations').textContent=d.stations;\n"
"            if(d.mode!==undefined){var m=document.getElementById('mode-tag');var l=document.getElementById('mode-label');\n"
"                if(d.mode==2){m.textContent='手机IMU';m.className='mode-tag mode-imu';l.textContent='手机 IMU';l.className='mode-tag mode-imu';}\n"
"                else if(d.mode==1){m.textContent='MPU6050';m.className='mode-tag mode-imu';l.textContent='MPU6050 IMU';l.className='mode-tag mode-imu';}\n"
"                else{m.textContent='手动';m.className='mode-tag mode-manual';l.textContent='手动模式';l.className='mode-tag mode-manual';}}\n"
"        }\n"
"        function pollStatus(){if(!wsOk){httpGet('/cmd',function(r){try{var d=JSON.parse(r);updateUI(d);}catch(x){}});}setTimeout(pollStatus,1000);}\n"
"        pollStatus();\n"
"        var wsOk=false;\n"
"        function httpGet(url,cb){var x=new XMLHttpRequest();x.open('GET',url,true);x.onload=function(){if(cb)cb(x.responseText);};x.send();}\n"
"        var lastPoll=0;\n"
"        function sendCmd(c,v){\n"
"            var u='/cmd?'+c+'_'+v;\n"
"            if(wsOk){ws.send(JSON.stringify({command:c,value:v}));httpGet(u);}else{httpGet(u);}\n"
"        }\n"
"        function sendServo(id,angle){\n"
"            var ch='XYZTEG'[id]||'X';\n"
"            var u='/cmd?'+ch+'_'+angle;\n"
"            if(wsOk){ws.send(JSON.stringify({command:'servo',id:id,angle:angle}));}\n"
"            httpGet(u);\n"
"        }\n"
"        function onNum(i){var v=parseInt(document.getElementById('si'+i).value);if(isNaN(v))return;var r=document.getElementById('s'+i);var mn=parseInt(r.min),mx=parseInt(r.max);if(v<mn)v=mn;if(v>mx)v=mx;r.value=v;document.getElementById('sv'+i).textContent=v;document.getElementById('si'+i).value=v;sendServo(i,v);}\n"
"        for(var i=0;i<5;i++){(function(id){\n"
"            var r=document.getElementById('s'+id);\n"
"            r.addEventListener('input',function(){var v=parseInt(r.value);document.getElementById('sv'+id).textContent=v;document.getElementById('si'+id).value=v;sendServo(id,v);});\n"
"        })(i);}\n"
"        conn();\n"
"    </script>\n"
"</body>\n"
"</html>\n";

/* ================================================================
 * HTTP Handlers
 * ================================================================ */

static servo_status_t s_last_status = {0};

static esp_err_t http_index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, strlen(index_html));
    return ESP_OK;
}

/* /cmd endpoint — mirrors ESP8266's command interface:
   GET /cmd         → returns current status JSON
   GET /cmd?X_90    → executes command, returns result        */
static esp_err_t http_cmd_handler(httpd_req_t *req)
{
    char buf[256];
    size_t query_len = httpd_req_get_url_query_len(req);

    if (query_len == 0 || query_len >= sizeof(buf)) {
        /* No args: return status JSON */
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "base", s_last_status.base_angle);
        cJSON_AddNumberToObject(root, "shoulder", s_last_status.shoulder_angle);
        cJSON_AddNumberToObject(root, "elbow", s_last_status.elbow_angle);
        cJSON_AddNumberToObject(root, "wrist", s_last_status.wrist_angle);
        cJSON_AddNumberToObject(root, "pitch", s_last_status.pitch);
        cJSON_AddNumberToObject(root, "roll", s_last_status.roll);
        cJSON_AddNumberToObject(root, "mode", s_last_status.mode);
        char *json = cJSON_PrintUnformatted(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, json, strlen(json));
        free(json);
        cJSON_Delete(root);
        return ESP_OK;
    }

    esp_err_t ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
        return ESP_FAIL;
    }

    /* Parse query: key=value or just key */
    char *eq = strchr(buf, '=');
    char *cmd_str = (eq && eq[1]) ? eq + 1 : buf;

    /* URL decode '_' back to '+' for relative commands like "X +10" */
    for (char *p = cmd_str; *p; p++) {
        if (*p == '_') *p = '+';
    }

    /* Split by ';' and execute each command */
    char *saveptr;
    char *token = strtok_r(cmd_str, ";", &saveptr);
    int count = 0;

    while (token && s_command_callback) {
        /* Trim leading/trailing spaces */
        while (*token == ' ') token++;
        char *end = token + strlen(token) - 1;
        while (end > token && *end == ' ') *end-- = '\0';

        if (*token) {
            /* Parse "X 90" or "X90" format */
            char axis[2] = {token[0], '\0'};
            float value = 0;
            char *val_str = token + 1;
            while (*val_str == ' ') val_str++;
            value = atof(val_str);

            /* Map single-letter axis names to servo commands */
            const char *servo_map = "XYZTEGH";
            const char *servo_cmds[] = {"servo0","servo1","servo2","servo3","servo4","","reset"};
            int idx = -1;
            for (int i = 0; servo_map[i]; i++) {
                if (axis[0] == servo_map[i]) { idx = i; break; }
            }
            if (idx >= 0 && servo_cmds[idx][0]) {
                s_command_callback(servo_cmds[idx], value);
                count++;
            } else if (axis[0] == 'H' || axis[0] == 'h') {
                s_command_callback("reset", 0);
                count++;
            } else if (axis[0] == 'W' || axis[0] == 'w') {
                s_command_callback("wave", 0);
                count++;
            } else if (axis[0] == 'M' || axis[0] == 'm') {
                s_command_callback("mode", value > 0.5f ? 1 : 0);
                count++;
            }
        }
        token = strtok_r(NULL, ";", &saveptr);
    }

    /* Return result */
    char result[64];
    snprintf(result, sizeof(result), "{\"ok\":%d}", count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, result, strlen(result));
    return ESP_OK;
}

/* ================================================================
 * Captive Portal DNS Server
 * ================================================================ */

static void dns_server_task(void *pv)
{
    (void)pv;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "DNS: socket failed"); vTaskDelete(NULL); return; }

    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(DNS_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS: bind failed"); close(sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "DNS server started on :53");

    uint8_t rx[256];
    struct sockaddr_in client;
    socklen_t cl = sizeof(client);
    while (1) {
        int len = recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&client, &cl);
        if (len < 12) continue;

        uint8_t resp[512];
        memcpy(resp, rx, len);
        resp[2] = 0x85; resp[3] = 0x80; /* QR=1 AA=1 RD=1 RA=1 */
        resp[7] = 0x01;                  /* ANCOUNT=1 */
        int p = len;
        resp[p++] = 0xC0; resp[p++] = 0x0C; /* pointer to question */
        resp[p++] = 0x00; resp[p++] = 0x01; /* TYPE=A */
        resp[p++] = 0x00; resp[p++] = 0x01; /* CLASS=IN */
        resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x00; resp[p++] = 0x3C; /* TTL=60 */
        resp[p++] = 0x00; resp[p++] = 0x04; /* RDLENGTH=4 */
        resp[p++] = 192; resp[p++] = 168; resp[p++] = 4; resp[p++] = 1; /* 192.168.4.1 */
        sendto(sock, resp, p, 0, (struct sockaddr *)&client, cl);
    }
    close(sock);
    vTaskDelete(NULL);
}

/* Captive portal redirect: unmatched paths → 302 to / */
static esp_err_t http_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<html><head><meta http-equiv=\"refresh\" content=\"0;url=/\"></head><body><a href=\"/\">OK</a></body></html>");
    return ESP_OK;
}

/* ================================================================
 * WebSocket client tracking
 * ================================================================ */

static void ws_client_add(int fd)
{
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] == -1) {
            ws_fds[i] = fd;
            ESP_LOGI(TAG, "WS client %d added (fd=%d)", i, fd);
            return;
        }
    }
    ESP_LOGW(TAG, "WS max clients reached, rejecting fd=%d", fd);
}

static void ws_client_remove(int fd)
{
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] == fd) {
            ws_fds[i] = -1;
            ESP_LOGI(TAG, "WS client %d removed", i);
            return;
        }
    }
}

/* ================================================================
 * WebSocket handler
 * ================================================================ */

static esp_err_t ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    if (fd < 0) return ESP_FAIL;

    ws_client_add(fd);

    uint8_t rx_buf[256];
    while (1) {
        /* Step 1: get frame info (len=0) */
        httpd_ws_frame_t ws_pkt = { .len = 0 };
        esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
        if (ret != ESP_OK) {
            ws_client_remove(fd);
            return ret;
        }

        if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) continue;
        if (ws_pkt.len == 0 || ws_pkt.len >= sizeof(rx_buf)) continue;

        /* Step 2: read actual payload */
        ws_pkt.payload = rx_buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ws_client_remove(fd);
            return ret;
        }
        rx_buf[ws_pkt.len] = '\0';

        cJSON *root = cJSON_Parse((char*)rx_buf);
        if (root) {
            cJSON *cmd = cJSON_GetObjectItem(root, "command");
            if (cJSON_IsString(cmd) && s_command_callback) {
                float value = 0;
                cJSON *val = cJSON_GetObjectItem(root, "value");
                if (cJSON_IsNumber(val)) value = (float)val->valuedouble;

                if (strcmp(cmd->valuestring, "servo") == 0) {
                    cJSON *id = cJSON_GetObjectItem(root, "id");
                    cJSON *angle = cJSON_GetObjectItem(root, "angle");
                    if (cJSON_IsNumber(id) && cJSON_IsNumber(angle)) {
                        char servo_cmd[16];
                        snprintf(servo_cmd, sizeof(servo_cmd), "servo%d", id->valueint);
                        s_command_callback(servo_cmd, (float)angle->valuedouble);
                    }
                } else if (strcmp(cmd->valuestring, "phone_imu") == 0) {
                    cJSON *pitch_j = cJSON_GetObjectItem(root, "pitch");
                    cJSON *roll_j  = cJSON_GetObjectItem(root, "roll");
                    if (cJSON_IsNumber(pitch_j) && cJSON_IsNumber(roll_j)) {
                        s_command_callback("phone_pitch", (float)pitch_j->valuedouble);
                        s_command_callback("phone_roll",  (float)roll_j->valuedouble);
                    }
                } else {
                    s_command_callback(cmd->valuestring, value);
                }
            }
            cJSON_Delete(root);
        }
    }
    return ESP_OK;
}

static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_index_handler,
    .user_ctx  = NULL,
};

static const httpd_uri_t ws_uri = {
    .uri            = "/ws",
    .method         = HTTP_GET,
    .handler        = ws_handler,
    .user_ctx       = NULL,
    .is_websocket   = true,
    .handle_ws_control_frames = true,
};

static const httpd_uri_t redirect_uri = {
    .uri       = "/*",
    .method    = HTTP_GET,
    .handler   = http_redirect_handler,
    .user_ctx  = NULL,
};

/* ================================================================
 * Public API
 * ================================================================ */

esp_err_t WebServer_Init(web_command_callback_t callback)
{
    s_command_callback = callback;
    memset(ws_fds, -1, sizeof(ws_fds));

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 12;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_LOGI(TAG, "Starting HTTP+WS server on port 80");
    esp_err_t ret = httpd_start(&s_http_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register / first (exact match takes priority) */
    ret = httpd_register_uri_handler(s_http_server, &index_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register / failed: %s", esp_err_to_name(ret));
        return ret;
    }

    {
        httpd_uri_t cmd_uri = { .uri = "/cmd", .method = HTTP_GET, .handler = http_cmd_handler };
        ret = httpd_register_uri_handler(s_http_server, &cmd_uri);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Register /cmd failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ret = httpd_register_uri_handler(s_http_server, &ws_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register /ws failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wildcard redirect: unmatched paths → 302 to / (captive portal trigger) */
    ret = httpd_register_uri_handler(s_http_server, &redirect_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register redirect failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start DNS capture server for captive portal */
    xTaskCreate(dns_server_task, "dns_srv", 3072, NULL, 3, &s_dns_task);

    ESP_LOGI(TAG, "Web server ready (DNS+HTTP+WS on :80)");
    return ESP_OK;
}

esp_err_t WebServer_Deinit(void)
{
    memset(ws_fds, -1, sizeof(ws_fds));

    if (s_dns_task) { vTaskDelete(s_dns_task); s_dns_task = NULL; }

    if (s_http_server) { httpd_stop(s_http_server); s_http_server = NULL; }

    ESP_LOGI(TAG, "Web server stopped");
    return ESP_OK;
}

void WebServer_SendStatus(const servo_status_t *status)
{
    if (!s_http_server || !status) return;

    /* Cache for /cmd endpoint */
    memcpy(&s_last_status, status, sizeof(s_last_status));

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddNumberToObject(root, "pitch", status->pitch);
    cJSON_AddNumberToObject(root, "roll", status->roll);
    cJSON_AddNumberToObject(root, "base", status->base_angle);
    cJSON_AddNumberToObject(root, "shoulder", status->shoulder_angle);
    cJSON_AddNumberToObject(root, "elbow", status->elbow_angle);
    cJSON_AddNumberToObject(root, "wrist", status->wrist_angle);
    cJSON_AddNumberToObject(root, "stations", status->num_stations);
    cJSON_AddNumberToObject(root, "mode", status->mode);

    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        cJSON_Delete(root);
        return;
    }

    size_t len = strlen(json_str);
    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)json_str,
        .len = len,
    };

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] != -1) {
            esp_err_t ret = httpd_ws_send_frame_async(s_http_server, ws_fds[i], &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "WS send fail fd=%d: %s", ws_fds[i], esp_err_to_name(ret));
                ws_fds[i] = -1;
            }
        }
    }

    free(json_str);
    cJSON_Delete(root);
}
