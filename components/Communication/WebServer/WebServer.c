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
#define WS_HEARTBEAT_INTERVAL_MS  3000  /* 客户端每3秒发一次心跳 */
#define WS_HEARTBEAT_TIMEOUT_MS   10000 /* 10秒无心跳则断开 */

static httpd_handle_t s_http_server = NULL;
static web_command_callback_t s_command_callback = NULL;
static int ws_fds[MAX_WS_CLIENTS] = {-1, -1, -1, -1};
static TickType_t ws_last_heartbeat[MAX_WS_CLIENTS] = {0, 0, 0, 0};
static TaskHandle_t s_dns_task = NULL;

static const char *index_html =
	"<!DOCTYPE html>\n"
	"<html lang=\"zh-CN\">\n"
	"<head>\n"
	"<meta charset=\"UTF-8\">\n"
	"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0,user-scalable=no\">\n"
	"<title>机械臂控制</title>\n"
	"<style>\n"
	"html,body{height:100%}\n"
	"body{height:100dvh;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:#f2f2f7;color:#1a1a1a;display:flex;flex-direction:column;overflow:hidden}\n"
	".topbar{display:flex;align-items:center;justify-content:space-between;padding:8px 12px;background:#fff;border-bottom:1px solid #e5e5ea;flex-shrink:0}\n"
	".topbar .conn{display:flex;align-items:center;gap:6px;font-size:13px;color:#666}\n"
	".topbar .conn .dot{width:8px;height:8px;border-radius:50%;flex-shrink:0}\n"
	".topbar .conn .dot.on{background:#34c759;box-shadow:0 0 6px rgba(52,199,89,.5);animation:pulse 2s infinite}\n"
	".topbar .conn .dot.off{background:#ff3b30}\n"
	"@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}\n"
	".topbar .mode-tag{padding:4px 10px;border-radius:12px;font-size:12px;font-weight:600;cursor:pointer;border:1px solid transparent;transition:all .15s}\n"
	".topbar .mode-tag:active{opacity:.7}\n"
	".topbar .mode-tag.manual{background:#fff3e0;color:#e65100;border-color:#ffcc80}\n"
	".topbar .mode-tag.imu{background:#e8f5e9;color:#2e7d32;border-color:#a5d6a7}\n"
	".tabs{display:flex;background:#fff;border-bottom:1px solid #e5e5ea;flex-shrink:0}\n"
	".tabs button{flex:1;padding:11px 4px;border:none;background:transparent;font-size:13px;font-weight:500;color:#8e8e93;cursor:pointer;position:relative;transition:color .2s}\n"
	".tabs button.active{color:#007aff;font-weight:600}\n"
	".tabs button.active::after{content:'';position:absolute;bottom:0;left:12px;right:12px;height:2px;background:#007aff;border-radius:1px}\n"
	".panels{flex:1;min-height:0;overflow-y:auto;-webkit-overflow-scrolling:touch}\n"
	".panel{display:none;padding:12px}\n"
	".panel.active{display:block}\n"
	".servo-card{background:#fff;border-radius:14px;padding:14px;box-shadow:0 1px 3px rgba(0,0,0,.06)}\n"
	".servo-row{display:flex;align-items:center;gap:4px;padding:10px 0}\n"
	".servo-row+.servo-row{border-top:1px solid #f2f2f7}\n"
	".servo-row .name{width:30px;font-size:12px;font-weight:600;color:#333;flex-shrink:0;text-align:center}\n"
	".servo-row .min,.servo-row .max{font-size:9px;color:#aaa;width:14px;text-align:center;flex-shrink:0}\n"
	".servo-row input[type=range]{flex:1;min-width:0;height:5px;-webkit-appearance:none;appearance:none;background:#e0e0e0;border-radius:3px;outline:none}\n"
	".servo-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:24px;height:24px;border-radius:50%;background:#fff;border:2px solid #007aff;box-shadow:0 1px 4px rgba(0,0,0,.15);cursor:pointer}\n"
	".servo-row input[type=range]:active::-webkit-slider-thumb{background:#007aff}\n"
	".servo-row .val{min-width:28px;font-size:15px;font-weight:700;color:#007aff;text-align:center}\n"
	".servo-row .btn-sm{width:24px;height:24px;border-radius:50%;border:1px solid #ddd;background:#fff;font-size:13px;font-weight:600;color:#666;cursor:pointer;flex-shrink:0;display:flex;align-items:center;justify-content:center;line-height:1;padding:0}\n"
	".servo-row .btn-sm:active{background:#007aff;color:#fff;border-color:#007aff}\n"
	".gripper-row .name{color:#ff6b35}\n"
	".gripper-row input[type=range]::-webkit-slider-thumb{border-color:#ff6b35}\n"
	".quick-btns{display:flex;gap:10px;margin-top:14px;flex-wrap:wrap}\n"
	".quick-btns button{flex:1;min-width:60px;padding:10px 6px;border:none;border-radius:10px;font-size:13px;font-weight:600;cursor:pointer;transition:all .15s}\n"
	".quick-btns .btn-home{background:#e8f0fe;color:#007aff}\n"
	".quick-btns .btn-wave{background:#fff3e0;color:#e65100}\n"
	".quick-btns .btn-save{background:#e8f5e9;color:#2e7d32}\n"
	".quick-btns .btn-grip-open{background:#ffe0e0;color:#c62828}\n"
	".quick-btns .btn-grip-close{background:#ffebee;color:#b71c1c}\n"
	".quick-btns button:active{opacity:.6;transform:scale(.97)}\n"
	".imu-phone{background:#fff;border-radius:14px;padding:16px;box-shadow:0 1px 3px rgba(0,0,0,.06);text-align:center;margin-bottom:12px}\n"
	".imu-phone .big-num{display:flex;justify-content:center;gap:28px}\n"
	".imu-phone .big-num div{display:flex;flex-direction:column;align-items:center}\n"
	".imu-phone .big-num .label{font-size:11px;color:#999;margin-bottom:4px}\n"
	".imu-phone .big-num .val{font-size:36px;font-weight:200;color:#333;line-height:1}\n"
	".imu-phone .big-num .val .deg{font-size:16px;color:#aaa}\n"
	".info-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:12px}\n"
	".info-cell{background:#fff;border-radius:10px;padding:10px;box-shadow:0 1px 3px rgba(0,0,0,.06)}\n"
	".info-cell .lbl{font-size:10px;color:#999;margin-bottom:2px}\n"
	".info-cell .v{font-size:15px;font-weight:600;color:#333}\n"
	".btn-sensor{width:100%;padding:12px;border:none;border-radius:12px;font-size:15px;font-weight:600;cursor:pointer;transition:all .15s;margin-bottom:8px}\n"
	".btn-sensor.start{background:#007aff;color:#fff}\n"
	".btn-sensor.stop{background:#ff3b30;color:#fff}\n"
	".btn-sensor:active{opacity:.7;transform:scale(.98)}\n"
	".sensor-status{text-align:center;font-size:12px;color:#999}\n"
	".sensor-status .ok{color:#34c759}\n"
	".coord-block{background:#fff;border-radius:14px;padding:14px;box-shadow:0 1px 3px rgba(0,0,0,.06)}\n"
	".coord-block+.coord-block{margin-top:12px}\n"
	".coord-block h3{font-size:12px;font-weight:600;color:#333;margin-bottom:10px;text-transform:uppercase;letter-spacing:.5px}\n"
	".coord-block.start h3{color:#007aff}\n"
	".coord-block.end h3{color:#ff6b35}\n"
	".coord-row{display:flex;align-items:center;gap:4px;margin-bottom:7px}\n"
	".coord-row .axis{width:14px;font-size:13px;font-weight:700;color:#999;text-align:center}\n"
	".coord-row input{flex:1;min-width:0;padding:9px 4px;border:1px solid #e0e0e0;border-radius:8px;font-size:15px;text-align:center;background:#f9f9f9;color:#333;outline:none}\n"
	".coord-row input:focus{border-color:#007aff;background:#fff}\n"
	".btn-pick{width:100%;padding:13px;border:none;border-radius:12px;font-size:15px;font-weight:600;cursor:pointer;transition:all .15s;margin-top:12px;background:#007aff;color:#fff}\n"
	".btn-pick:active{opacity:.7;transform:scale(.98)}\n"
	".bottombar{display:flex;align-items:center;gap:0;background:rgba(255,255,255,.92);backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);border-top:1px solid #e5e5ea;padding:6px 2px;padding-bottom:max(6px,env(safe-area-inset-bottom));flex-shrink:0;overflow-x:auto}\n"
	".bottombar .item{flex:1;min-width:48px;text-align:center;font-size:10px;color:#999;line-height:1.2}\n"
	".bottombar .item .v{font-size:14px;font-weight:700;color:#333}\n"
	".bottombar .item.grip .v{color:#ff6b35}\n"
	".bottombar .item.grip .v .dot{display:inline-block;width:5px;height:5px;border-radius:50%;margin-left:2px;vertical-align:middle}\n"
	".bottombar .item.grip .v .dot.closed{background:#ff3b30}\n"
	".bottombar .item.grip .v .dot.open{background:#34c759}\n"
	"#firefox-hint{display:none;background:#fff3cd;color:#856404;text-align:center;padding:8px 12px;font-size:12px;font-weight:500;flex-shrink:0;line-height:1.4}\n"
	"#firefox-hint code{background:rgba(0,0,0,.07);padding:1px 4px;border-radius:3px;font-size:11px}\n"
	"@media(min-width:420px){\n"
	"  body{max-width:420px;margin:0 auto;border-left:1px solid #e5e5ea;border-right:1px solid #e5e5ea}\n"
	"  .servo-row .name{width:36px;font-size:13px}\n"
	"  .servo-row .btn-sm{width:28px;height:28px;font-size:15px}\n"
	"  .servo-row .val{min-width:34px;font-size:16px}\n"
	"  .servo-row .min,.servo-row .max{width:16px;font-size:10px}\n"
	"  .tabs button{font-size:14px;padding:11px 8px}\n"
	"}\n"
	"</style>\n"
	"</head>\n"
	"<body>\n"
	"<div id=\"firefox-hint\"></div>\n"
	"<div class=\"topbar\">\n"
	"  <div class=\"conn\"><span class=\"dot off\" id=\"conn-dot\"></span></div>\n"
	"  <span class=\"mode-tag manual\" id=\"mode-tag\" onclick=\"toggleMode()\">手动模式</span>\n"
	"  <span style=\"font-size:12px;color:#aaa\" id=\"stations\">0人</span>\n"
	"</div>\n"
	"<div class=\"tabs\">\n"
	"  <button type=\"button\" class=\"active\" data-panel=\"servo\">舵机控制</button>\n"
	"  <button type=\"button\" data-panel=\"imu\">体感 PID</button>\n"
	"  <button type=\"button\" data-panel=\"pick\">坐标取放</button>\n"
	"</div>\n"
	"<div class=\"panels\">\n"
	"  <div class=\"panel active\" id=\"panel-servo\">\n"
	"    <div class=\"servo-card\">\n"
	"      <div class=\"servo-row\"><span class=\"name\">底座</span><span class=\"min\">0</span><input type=\"range\" id=\"s0\" min=\"0\" max=\"180\" value=\"90\"><span class=\"max\">180</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(0,-1)\">-</button><span class=\"val\" id=\"sv0\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(0,1)\">+</button></div>\n"
	"      <div class=\"servo-row\"><span class=\"name\">肩</span><span class=\"min\">60</span><input type=\"range\" id=\"s1\" min=\"60\" max=\"120\" value=\"90\"><span class=\"max\">120</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(1,-1)\">-</button><span class=\"val\" id=\"sv1\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(1,1)\">+</button></div>\n"
	"      <div class=\"servo-row\"><span class=\"name\">肘</span><span class=\"min\">40</span><input type=\"range\" id=\"s2\" min=\"40\" max=\"140\" value=\"90\"><span class=\"max\">140</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(2,-1)\">-</button><span class=\"val\" id=\"sv2\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(2,1)\">+</button></div>\n"
	"      <div class=\"servo-row\"><span class=\"name\">腕</span><span class=\"min\">20</span><input type=\"range\" id=\"s3\" min=\"20\" max=\"160\" value=\"90\"><span class=\"max\">160</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(3,-1)\">-</button><span class=\"val\" id=\"sv3\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(3,1)\">+</button></div>\n"
	"      <div class=\"servo-row gripper-row\"><span class=\"name\">夹爪</span><span class=\"min\">0</span><input type=\"range\" id=\"s4\" min=\"0\" max=\"90\" value=\"90\"><span class=\"max\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(4,-1)\">-</button><span class=\"val\" id=\"sv4\">90</span><button type=\"button\" class=\"btn-sm\" onclick=\"adj(4,1)\">+</button></div>\n"
	"    </div>\n"
	"    <div class=\"quick-btns\">\n"
	"      <button type=\"button\" class=\"btn-home\" onclick=\"sendCmd('reset',0);resetUI()\">归位</button>\n"
	"      <button type=\"button\" class=\"btn-wave\" onclick=\"sendCmd('wave',0)\">挥手</button>\n"
	"    </div>\n"
	"  </div>\n"
	"  <div class=\"panel\" id=\"panel-imu\">\n"
	"    <div class=\"imu-phone\">\n"
	"      <div class=\"big-num\">\n"
	"        <div><span class=\"label\">Pitch</span><span class=\"val\"><span id=\"phone-pitch\">--</span><span class=\"deg\">°</span></span></div>\n"
	"        <div><span class=\"label\">Roll</span><span class=\"val\"><span id=\"phone-roll\">--</span><span class=\"deg\">°</span></span></div>\n"
	"      </div>\n"
	"    </div>\n"
	"    <div class=\"info-grid\">\n"
	"      <div class=\"info-cell\"><div class=\"lbl\">目标 Pitch</div><div class=\"v\" id=\"tgt-p\">--</div></div>\n"
	"      <div class=\"info-cell\"><div class=\"lbl\">实际 Pitch</div><div class=\"v\" id=\"actual-p\">0.0°</div></div>\n"
	"      <div class=\"info-cell\"><div class=\"lbl\">PID 误差</div><div class=\"v\" id=\"pid-pe\" style=\"color:#ff9500\">0.0°</div></div>\n"
	"      <div class=\"info-cell\"><div class=\"lbl\">PID 修正</div><div class=\"v\" id=\"pid-po\" style=\"color:#34c759\">0.0°</div></div>\n"
	"    </div>\n"
	"    <button type=\"button\" class=\"btn-sensor start\" id=\"sensor-btn\" onclick=\"startSensor()\">启动传感器</button>\n"
	"    <div class=\"sensor-status\" id=\"imu-perm\">点击按钮启动</div>\n"
	"    <div class=\"quick-btns\" style=\"margin-top:12px\">\n"
	"      <button type=\"button\" class=\"btn-grip-open\" onclick=\"sendCmd('gripper_open',0);gripQuick(0)\">夹爪张开</button>\n"
	"      <button type=\"button\" class=\"btn-grip-close\" onclick=\"sendCmd('gripper_close',0);gripQuick(90)\">夹爪闭合</button>\n"
	"    </div>\n"
	"  </div>\n"
	"  <div class=\"panel\" id=\"panel-pick\">\n"
	"    <div class=\"coord-block start\">\n"
	"      <h3>起点</h3>\n"
	"      <div class=\"coord-row\"><span class=\"axis\">X</span><input type=\"number\" id=\"ksx\" value=\"80\" step=\"1\"><span class=\"axis\">Y</span><input type=\"number\" id=\"ksy\" value=\"0\" step=\"1\"><span class=\"axis\">Z</span><input type=\"number\" id=\"ksz\" value=\"0\" step=\"1\"></div>\n"
	"    </div>\n"
	"    <div class=\"coord-block end\">\n"
	"      <h3>终点</h3>\n"
	"      <div class=\"coord-row\"><span class=\"axis\">X</span><input type=\"number\" id=\"kex\" value=\"0\" step=\"1\"><span class=\"axis\">Y</span><input type=\"number\" id=\"key\" value=\"80\" step=\"1\"><span class=\"axis\">Z</span><input type=\"number\" id=\"kez\" value=\"0\" step=\"1\"></div>\n"
	"    </div>\n"
	"    <button type=\"button\" class=\"btn-pick\" onclick=\"kinematicMove()\">执行取放</button>\n"
	"    <div class=\"quick-btns\" style=\"margin-top:10px\">\n"
	"      <button type=\"button\" class=\"btn-save\" onclick=\"saveCoordPreset()\">保存坐标</button>\n"
	"    </div>\n"
	"  </div>\n"
	"</div>\n"
	"<div class=\"bottombar\" id=\"bottombar\">\n"
	"  <div class=\"item\"><div class=\"v\" id=\"bb-base\">90</div>底座</div>\n"
	"  <div class=\"item\"><div class=\"v\" id=\"bb-shoulder\">90</div>肩</div>\n"
	"  <div class=\"item\"><div class=\"v\" id=\"bb-elbow\">90</div>肘</div>\n"
	"  <div class=\"item\"><div class=\"v\" id=\"bb-wrist\">90</div>腕</div>\n"
	"  <div class=\"item grip\"><div class=\"v\"><span id=\"bb-gripper\">90</span><span class=\"dot closed\" id=\"grip-dot\"></span></div>夹爪</div>\n"
	"</div>\n"
	"<script>\n"
	"var ws=null,rt=null,phonePitch=0,phoneRoll=0,imuReady=false,imuActive=false,imuType='',wsOk=false;\n"
	"var gMode=0;\n"
	"var g=function(i){return g._[i]||(g._[i]=document.getElementById(i))};g._={};\n"
	"(function(){\n"
	"  var btns=document.querySelectorAll('.tabs button');\n"
	"  for(var i=0;i<btns.length;i++){\n"
	"    btns[i].addEventListener('click',function(){\n"
	"      var panelId=this.getAttribute('data-panel');\n"
	"      for(var j=0;j<btns.length;j++)btns[j].classList.remove('active');\n"
	"      this.classList.add('active');\n"
	"      var panels=document.querySelectorAll('.panel');\n"
	"      for(var k=0;k<panels.length;k++)panels[k].classList.remove('active');\n"
	"      var target=document.getElementById('panel-'+panelId);\n"
	"      if(target)target.classList.add('active');\n"
	"    });\n"
	"  }\n"
	"})();\n"
	"function toggleMode(){\n"
	"  gMode=gMode?0:1;\n"
	"  sendCmd('mode',gMode);\n"
	"  setModeUI(gMode)\n"
	"}\n"
	"function setModeUI(m){\n"
	"  gMode=m;\n"
	"  var t=document.getElementById('mode-tag');\n"
	"  if(m==1){\n"
	"    t.textContent='体感PID';t.className='mode-tag imu';\n"
	"  }else{\n"
	"    t.textContent='手动模式';t.className='mode-tag manual';\n"
	"  }\n"
	"}\n"
	"function gripQuick(v){\n"
	"  document.getElementById('s4').value=v;document.getElementById('sv4').textContent=v;\n"
	"  updateBottomBar(4,v)\n"
	"}\n"
	"function adj(id,d){\n"
	"  var r=document.getElementById('s'+id);if(!r)return;\n"
	"  var v=parseInt(r.value)+d;\n"
	"  var mn=parseInt(r.min)||0,mx=parseInt(r.max)||180;\n"
	"  if(v<mn)v=mn;if(v>mx)v=mx;\n"
	"  r.value=v;document.getElementById('sv'+id).textContent=v;\n"
	"  updateBottomBar(id,v);\n"
	"  if(sliderCool[id]){clearTimeout(sliderCool[id])}\n"
	"  sliderCool[id]=setTimeout(function(){sliderCool[id]=0},2000);\n"
	"  sendServo(id,v)\n"
	"}\n"
	"var sliderDrag={},sliderCool={};\n"
	"var lastManualSet={};\n"
	"(function(){\n"
	"  for(var i=0;i<5;i++){\n"
	"    (function(id){\n"
	"      var r=document.getElementById('s'+id);if(!r)return;\n"
	"      r.addEventListener('input',function(){var v=parseInt(this.value);document.getElementById('sv'+id).textContent=v;updateBottomBar(id,v)});\n"
	"      r.addEventListener('pointerdown',function(){sliderDrag[id]=true;if(sliderCool[id]){clearTimeout(sliderCool[id]);sliderCool[id]=0}});\n"
	"      r.addEventListener('pointerup',function(){var v=parseInt(this.value);sliderDrag[id]=false;sendServo(id,v);lastManualSet[id]=v;sliderCool[id]=setTimeout(function(){sliderCool[id]=0},2000)});\n"
	"      r.addEventListener('pointerleave',function(){if(sliderDrag[id]){var v=parseInt(this.value);sliderDrag[id]=false;sendServo(id,v);lastManualSet[id]=v;sliderCool[id]=setTimeout(function(){sliderCool[id]=0},2000)}});\n"
	"      r.addEventListener('change',function(){if(!sliderDrag[id])sendServo(id,parseInt(this.value))});\n"
	"    })(i)\n"
	"  }\n"
	"})();\n"
	"function updateBottomBar(id,v){\n"
	"  var ids=['bb-base','bb-shoulder','bb-elbow','bb-wrist','bb-gripper'];\n"
	"  if(ids[id])document.getElementById(ids[id]).textContent=v;\n"
	"  if(id===4){\n"
	"    var d=document.getElementById('grip-dot');\n"
	"    if(d)d.className='dot '+(v>45?'closed':'open');\n"
	"  }\n"
	"}\n"
	"function resetUI(){\n"
	"  for(var i=0;i<5;i++){\n"
	"    var v=90;\n"
	"    document.getElementById('s'+i).value=v;document.getElementById('sv'+i).textContent=v;\n"
	"    updateBottomBar(i,v);\n"
	"    if(sliderCool[i]){clearTimeout(sliderCool[i])}\n"
	"    sliderCool[i]=setTimeout(function(){sliderCool[i]=0},2000)\n"
	"  }\n"
	"}\n"
	"function saveCoordPreset(){\n"
	"  var s='sx='+document.getElementById('ksx').value+'&sy='+document.getElementById('ksy').value+'&sz='+document.getElementById('ksz').value;\n"
	"  var e='ex='+document.getElementById('kex').value+'&ey='+document.getElementById('key').value+'&ez='+document.getElementById('kez').value;\n"
	"  localStorage.setItem('coord_preset',s+'&'+e);alert('坐标已保存')\n"
	"}\n"
	"(function(){\n"
	"  var c=localStorage.getItem('coord_preset');\n"
	"  if(c){\n"
	"    var m=c.split('&');\n"
	"    if(m.length>=6){\n"
	"      document.getElementById('ksx').value=m[0].split('=')[1]||80;\n"
	"      document.getElementById('ksy').value=m[1].split('=')[1]||0;\n"
	"      document.getElementById('ksz').value=m[2].split('=')[1]||0;\n"
	"      document.getElementById('kex').value=m[3].split('=')[1]||0;\n"
	"      document.getElementById('key').value=m[4].split('=')[1]||80;\n"
	"      document.getElementById('kez').value=m[5].split('=')[1]||0\n"
	"    }\n"
	"  }\n"
	"})();\n"
	"function onOrient(e){\n"
	"  if(e.beta===null&&e.gamma===null)return;\n"
	"  phonePitch=e.beta||0;phoneRoll=e.gamma||0;\n"
	"  document.getElementById('phone-pitch').textContent=phonePitch.toFixed(1);\n"
	"  document.getElementById('phone-roll').textContent=phoneRoll.toFixed(1)\n"
	"}\n"
	"function onMotion(e){\n"
	"  if(imuType)return;\n"
	"  var a=e.accelerationIncludingGravity;if(!a)return;\n"
	"  phonePitch=Math.atan2(a.x,Math.sqrt(a.y*a.y+a.z*a.z))*180/Math.PI;\n"
	"  phoneRoll=Math.atan2(a.y,Math.sqrt(a.x*a.x+a.z*a.z))*180/Math.PI;\n"
	"  document.getElementById('phone-pitch').textContent=phonePitch.toFixed(1);\n"
	"  document.getElementById('phone-roll').textContent=phoneRoll.toFixed(1)\n"
	"}\n"
	"function startSensor(){\n"
	"  if(imuActive){\n"
	"    imuActive=false;imuReady=false;\n"
	"    window.removeEventListener('deviceorientation',onOrient);\n"
	"    window.removeEventListener('devicemotion',onMotion);\n"
	"    document.getElementById('imu-perm').textContent='已停止';\n"
	"    var b=document.getElementById('sensor-btn');\n"
	"    b.textContent='启动传感器';b.className='btn-sensor start';b.disabled=false;\n"
	"    document.getElementById('phone-pitch').textContent='--';\n"
	"    document.getElementById('phone-roll').textContent='--';\n"
	"    document.getElementById('firefox-hint').style.display='none';\n"
	"    return\n"
	"  }\n"
	"  imuActive=true;\n"
	"  var permEl=document.getElementById('imu-perm');\n"
	"  var btn=document.getElementById('sensor-btn');\n"
	"  permEl.textContent='检测中...';\n"
	"  var isFirefox=navigator.userAgent.toLowerCase().indexOf('firefox')>-1;\n"
	"  if(typeof DeviceOrientationEvent!=='undefined'&&typeof DeviceOrientationEvent.requestPermission==='function'){\n"
	"    DeviceOrientationEvent.requestPermission().then(function(s){\n"
	"      if(s==='granted'){\n"
	"        window.addEventListener('deviceorientation',onOrient);\n"
	"        imuReady=true;imuType='orient';\n"
	"        permEl.innerHTML='<span class=ok>已授权</span>';\n"
	"        btn.textContent='停止传感器';btn.className='btn-sensor stop';btn.disabled=false\n"
	"      }else{permEl.textContent='被拒绝';imuActive=false}\n"
	"    }).catch(function(){permEl.textContent='权限错误';imuActive=false});\n"
	"    return\n"
	"  }\n"
	"  var orientHandler=function(e){\n"
	"    if(e.beta!==null||e.gamma!==null){\n"
	"      if(!imuReady){\n"
	"        imuReady=true;imuType='orient';\n"
	"        permEl.innerHTML='<span class=ok>已启动</span>';\n"
	"        btn.textContent='停止传感器';btn.className='btn-sensor stop';btn.disabled=false\n"
	"      }\n"
	"      onOrient(e)\n"
	"    }\n"
	"  };\n"
	"  var motionHandler=function(e){\n"
	"    if(!imuReady&&e.accelerationIncludingGravity&&e.accelerationIncludingGravity.x!==null){\n"
	"      imuReady=true;imuType='accel';\n"
	"      permEl.innerHTML='<span class=ok>已启动(重力)</span>';\n"
	"      btn.textContent='停止传感器';btn.className='btn-sensor stop';btn.disabled=false\n"
	"    }\n"
	"    if(!imuReady||imuType==='accel')onMotion(e)\n"
	"  };\n"
	"  window.addEventListener('deviceorientation',orientHandler);\n"
	"  window.addEventListener('devicemotion',motionHandler);\n"
	"  if(isFirefox&&window.DeviceOrientationEvent){\n"
	"    try{window.dispatchEvent(new DeviceOrientationEvent('deviceorientation',{beta:0,gamma:0}))}catch(e){}\n"
	"  }\n"
	"  setTimeout(function(){\n"
	"    if(!imuReady){\n"
	"      permEl.textContent='不支持';\n"
	"      var hint=document.getElementById('firefox-hint');\n"
	"      hint.innerHTML='<b>传感器不可用？</b> 复制 <b>192.168.4.1</b> 到 <b>Firefox 浏览器</b> 打开';\n"
	"      hint.style.display='block';\n"
	"      imuActive=false\n"
	"    }\n"
	"  },3000)\n"
	"}\n"
	"setInterval(function(){\n"
	"  if(wsOk&&imuReady)ws.send(JSON.stringify({command:'phone_imu',pitch:Math.round(phonePitch*10)/10,roll:Math.round(phoneRoll*10)/10}));\n"
	"  else if(!wsOk&&imuReady){httpGet('/cmd?phone_pitch_'+Math.round(phonePitch*10)/10);httpGet('/cmd?phone_roll_'+Math.round(phoneRoll*10)/10)}\n"
	"},100);\n"
	"setInterval(function(){if(wsOk)ws.send(JSON.stringify({command:'heartbeat'}))},3000);\n"
	"var wsReconnectDelay=3000,wsMaxDelay=30000;\n"
	"function conn(){\n"
	"  if(ws&&ws.readyState!==WebSocket.CLOSED)return;\n"
	"  ws=new WebSocket('ws://'+window.location.host+'/ws');\n"
	"  ws.onopen=function(){\n"
	"    wsOk=true;wsReconnectDelay=3000;\n"
	"    document.getElementById('conn-dot').className='dot on';\n"
	"  };\n"
	"  ws.onclose=function(){\n"
	"    wsOk=false;\n"
	"    document.getElementById('conn-dot').className='dot off';\n"
	"    if(rt)clearTimeout(rt);\n"
	"    rt=setTimeout(conn,wsReconnectDelay);\n"
	"    wsReconnectDelay=Math.min(wsReconnectDelay*1.5,wsMaxDelay)\n"
	"  };\n"
	"  ws.onerror=function(){wsOk=false;if(ws)ws.close()};\n"
	"  ws.onmessage=function(e){try{var d=JSON.parse(e.data);updateUI(d)}catch(x){}}\n"
	"}\n"
	"document.addEventListener('visibilitychange',function(){\n"
	"  if(!document.hidden&&(!ws||ws.readyState!==WebSocket.OPEN)){\n"
	"    if(rt)clearTimeout(rt);wsReconnectDelay=3000;conn()\n"
	"  }\n"
	"});\n"
	"window.addEventListener('focus',function(){\n"
	"  if(!ws||ws.readyState!==WebSocket.OPEN){\n"
	"    if(rt)clearTimeout(rt);wsReconnectDelay=3000;conn()\n"
	"  }\n"
	"});\n"
	"function updateUI(d){\n"
	"  if(d.pitch!==undefined)g('actual-p').textContent=d.pitch.toFixed(1)+'°';\n"
	"  if(d.base!==undefined){g('bb-base').textContent=d.base.toFixed(0);if(!sliderDrag[0]&&!sliderCool[0]){var v0=Math.round(d.base);g('s0').value=v0;g('sv0').textContent=v0}}\n"
	"  if(d.shoulder!==undefined){g('bb-shoulder').textContent=d.shoulder.toFixed(0);if(!sliderDrag[1]&&!sliderCool[1]){var v1=Math.round(d.shoulder);g('s1').value=v1;g('sv1').textContent=v1}}\n"
	"  if(d.elbow!==undefined){g('bb-elbow').textContent=d.elbow.toFixed(0);if(!sliderDrag[2]&&!sliderCool[2]){var v2=Math.round(d.elbow);g('s2').value=v2;g('sv2').textContent=v2}}\n"
	"  if(d.wrist!==undefined){g('bb-wrist').textContent=d.wrist.toFixed(0);if(!sliderDrag[3]&&!sliderCool[3]){var v3=Math.round(d.wrist);g('s3').value=v3;g('sv3').textContent=v3}}\n"
	"  if(d.gripper!==undefined){\n"
	"    g('bb-gripper').textContent=d.gripper.toFixed(0);\n"
	"    var gd=g('grip-dot');\n"
	"    if(gd){var gv=Math.round(d.gripper);gv<10?gd.className='dot open':gd.className='dot closed'}\n"
	"    if(!sliderDrag[4]&&!sliderCool[4]){var v4=Math.round(d.gripper);g('s4').value=v4;g('sv4').textContent=v4}\n"
	"  }\n"
	"  if(d.stations!==undefined)g('stations').textContent=d.stations>0?d.stations+'人':'0人';\n"
	"  if(d.tgt_p!==undefined)g('tgt-p').textContent=d.tgt_p.toFixed(1)+'°';\n"
	"  if(d.pid_pe!==undefined)g('pid-pe').textContent=d.pid_pe.toFixed(2)+'°';\n"
	"  if(d.pid_po!==undefined)g('pid-po').textContent=d.pid_po.toFixed(2)+'°';\n"
	"  if(d.mode!==undefined){setModeUI(d.mode)}\n"
	"}\n"
	"function pollStatus(){\n"
	"  if(!wsOk){httpGet('/cmd',function(r){try{var d=JSON.parse(r);updateUI(d)}catch(x){}})}\n"
	"  setTimeout(pollStatus,1000)\n"
	"}\n"
	"pollStatus();\n"
	"function httpGet(url,cb){var x=new XMLHttpRequest();x.open('GET',url,true);x.onload=function(){if(cb)cb(x.responseText)};x.send()}\n"
	"function sendCmd(c,v){\n"
	"  var u='/cmd?'+c+'_'+v;\n"
	"  if(wsOk){ws.send(JSON.stringify({command:c,value:v}))}else{httpGet(u)}\n"
	"}\n"
	"function sendServo(id,angle){\n"
	"  var u='/cmd?'+'XYZTE'[id]+'_'+angle;\n"
	"  if(wsOk){ws.send(JSON.stringify({command:'servo',id:id,angle:angle}))}else{httpGet(u)}\n"
	"}\n"
	"function kinematicMove(){\n"
	"  var sx=parseFloat(document.getElementById('ksx').value)||0;\n"
	"  var sy=parseFloat(document.getElementById('ksy').value)||0;\n"
	"  var sz=parseFloat(document.getElementById('ksz').value)||0;\n"
	"  var ex=parseFloat(document.getElementById('kex').value)||0;\n"
	"  var ey=parseFloat(document.getElementById('key').value)||0;\n"
	"  var ez=parseFloat(document.getElementById('kez').value)||0;\n"
	"  if(wsOk){ws.send(JSON.stringify({command:'kinematic_move',sx:sx,sy:sy,sz:sz,ex:ex,ey:ey,ez:ez}))}\n"
	"  else{httpGet('/cmd?kmove='+sx+','+sy+','+sz+','+ex+','+ey+','+ez)}\n"
	"}\n"
	"conn();\n"
	"</script>\n"
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
        if (json) {
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, json, strlen(json));
            free(json);
        }
        cJSON_Delete(root);
        return ESP_OK;
    }

    esp_err_t ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
        return ESP_FAIL;
    }

    /* kmove= 格式：坐标驱动的HTTP回退（避免 _→+ 转换破坏负号） */
    if (strncmp(buf, "kmove=", 6) == 0) {
        float c[6];
        if (sscanf(buf + 6, "%f,%f,%f,%f,%f,%f",
                   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5]) == 6) {
            char cmd[96];
            snprintf(cmd, sizeof(cmd), "kmove %.1f %.1f %.1f %.1f %.1f %.1f",
                     c[0], c[1], c[2], c[3], c[4], c[5]);
            if (s_command_callback) s_command_callback(cmd, 0);
        }
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"ok\":1}", 7);
        return ESP_OK;
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
            /* ── 命名命令（支持 Web UI sendCmd 的 HTTP 回退）── */
            bool handled = false;

            /* reset / wave — 无参数 */
            if (strncmp(token, "reset", 5) == 0 && (token[5] == '+' || token[5] == '\0')) {
                s_command_callback("reset", 0); handled = true;
            } else if (strncmp(token, "wave", 4) == 0 && (token[4] == '+' || token[4] == '\0')) {
                s_command_callback("wave", 0); handled = true;
            }
            /* mode — 带参数 */
            else if (strncmp(token, "mode", 4) == 0 && (token[4] == '+' || token[4] == '\0')) {
                s_command_callback("mode", atof(token + 5));
                handled = true;
            }
            /* gripper_open / gripper_close */
            else if (strncmp(token, "gripper+open", 12) == 0) {
                s_command_callback("gripper_open", 0); handled = true;
            } else if (strncmp(token, "gripper+close", 13) == 0) {
                s_command_callback("gripper_close", 0); handled = true;
            }
            /* kinematic_move */
            else if (strncmp(token, "kinematic+move", 14) == 0) {
                s_command_callback("kinematic_move", 0); handled = true;
            }
            /* phone_pitch / phone_roll（WebSocket断连时的IMU回退）
             * URL格式 phone_pitch_-12.3 经 _→+ 转换为 phone+pitch+-12.3
             * 跳过命令名后的 + 再解析数值，避免 atof("+-12.3") 返回 0 */
            else if (strncmp(token, "phone+pitch", 11) == 0) {
                const char *v = token + 11;
                if (*v == '+') v++;
                s_command_callback("phone_pitch", atof(v));
                handled = true;
            } else if (strncmp(token, "phone+roll", 10) == 0) {
                const char *v = token + 10;
                if (*v == '+') v++;
                s_command_callback("phone_roll", atof(v));
                handled = true;
            }

            if (handled) {
                count++;
            } else {
                /* ── 单字母命令回退 ── */
                char axis[2] = {token[0], '\0'};
                float value = 0;
                char *val_str = token + 1;
                while (*val_str == ' ') val_str++;
                value = atof(val_str);

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
 * Forward declarations
 * ================================================================ */

static void ws_heartbeat_check_task(void *pvParameters);

/* ================================================================
 * WebSocket client tracking
 * ================================================================ */

static void ws_client_add(int fd)
{
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (ws_fds[i] == -1) {
            ws_fds[i] = fd;
            ws_last_heartbeat[i] = xTaskGetTickCount();
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
            ws_last_heartbeat[i] = 0;
            ESP_LOGI(TAG, "WS client %d removed", i);
            return;
        }
    }
}

static void ws_client_remove_by_index(int idx)
{
    if (idx < 0 || idx >= MAX_WS_CLIENTS) return;
    if (ws_fds[idx] != -1) {
        /* 不直接close socket，让httpd自己管理连接生命周期
         * 直接close会导致httpd内部状态不一致，引发fd泄漏 */
        ws_fds[idx] = -1;
        ws_last_heartbeat[idx] = 0;
        ESP_LOGI(TAG, "WS client %d marked invalid", idx);
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

                /* 心跳包：更新最后活动时间 */
                if (strcmp(cmd->valuestring, "heartbeat") == 0) {
                    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                        if (ws_fds[i] == fd) {
                            ws_last_heartbeat[i] = xTaskGetTickCount();
                            break;
                        }
                    }
                }
                else if (strcmp(cmd->valuestring, "servo") == 0) {
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
                } else if (strcmp(cmd->valuestring, "kinematic_move") == 0) {
                    /* 解析坐标参数：sx/sy/sz 起点, ex/ey/ez 终点 */
                    cJSON *sx = cJSON_GetObjectItem(root, "sx");
                    cJSON *sy = cJSON_GetObjectItem(root, "sy");
                    cJSON *sz = cJSON_GetObjectItem(root, "sz");
                    cJSON *ex = cJSON_GetObjectItem(root, "ex");
                    cJSON *ey = cJSON_GetObjectItem(root, "ey");
                    cJSON *ez = cJSON_GetObjectItem(root, "ez");
                    if (cJSON_IsNumber(sx) && cJSON_IsNumber(sy) && cJSON_IsNumber(sz) &&
                        cJSON_IsNumber(ex) && cJSON_IsNumber(ey) && cJSON_IsNumber(ez)) {
                        char buf[96];
                        snprintf(buf, sizeof(buf), "kmove %.1f %.1f %.1f %.1f %.1f %.1f",
                                 sx->valuedouble, sy->valuedouble, sz->valuedouble,
                                 ex->valuedouble, ey->valuedouble, ez->valuedouble);
                        s_command_callback(buf, 0);
                    } else {
                        s_command_callback("kinematic_move", 0);
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
    config.max_open_sockets = 4;       /* LWIP_MAX_SOCKETS=10, httpd内部用3个，留4个给WS/HTTP */
    config.lru_purge_enable = true;    /* 启用LRU淘汰，自动关闭最久未用的连接 */
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

    /* Start WebSocket heartbeat timeout check task */
    xTaskCreate(ws_heartbeat_check_task, "ws_hb_chk", 2048, NULL, 4, NULL);

    ESP_LOGI(TAG, "Web server ready (DNS+HTTP+WS on :80)");
    return ESP_OK;
}

/* ================================================================
 * WebSocket 心跳超时检测任务
 * ================================================================ */

static void ws_heartbeat_check_task(void *pvParameters)
{
    (void)pvParameters;

    while (1)
    {
        TickType_t now = xTaskGetTickCount();

        for (int i = 0; i < MAX_WS_CLIENTS; i++)
        {
            if (ws_fds[i] == -1) continue;

            /* 检查超时：超过10秒未收到客户端心跳则断开 */
            if (ws_last_heartbeat[i] != 0 &&
                (now - ws_last_heartbeat[i]) * portTICK_PERIOD_MS > WS_HEARTBEAT_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "WS client %d heartbeat timeout (fd=%d)", i, ws_fds[i]);
                ws_client_remove_by_index(i);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(WS_HEARTBEAT_INTERVAL_MS));
    }
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
    cJSON_AddNumberToObject(root, "gripper", status->gripper_angle);
    cJSON_AddNumberToObject(root, "stations", status->num_stations);
    cJSON_AddNumberToObject(root, "mode", status->mode);

    /* PID反馈数据（仅pitch有闭环，roll直接映射底座） */
    cJSON_AddNumberToObject(root, "tgt_p", status->target_pitch);
    cJSON_AddNumberToObject(root, "tgt_r", status->target_roll);
    cJSON_AddNumberToObject(root, "pid_pe", status->pid_pitch_error);
    cJSON_AddNumberToObject(root, "pid_po", status->pid_pitch_output);

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
                /* 先复制 fd，避免 ws_client_remove 修改数组后索引失效 */
                int bad_fd = ws_fds[i];
                ws_fds[i] = -1;
                ws_last_heartbeat[i] = 0;
                ESP_LOGI(TAG, "WS client fd=%d marked invalid", bad_fd);
            }
        }
    }

    free(json_str);
    cJSON_Delete(root);
}
