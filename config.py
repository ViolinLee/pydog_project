# -*- coding: utf-8 -*-

# from pydog.utils import load_panel_html
from micropython import const

# Physical Attributes
l1 = const(80)
l2 = const(80)
body_l = const(165)
body_w = const(122)

calibrate_servos_pos = (90, 90, 90, 90, 90, 90, 90, 90)  # Assembly Features
calibrate_joints_angles = (0, 90, 0, 90, 0, 90, 0, 90)  # Kinematics Model
calibration_txt = "pydog/calibration.txt"
init_joints_angles = (40, 110, 40, 110, 40, 110, 40, 110)

# IOs Mappings
joints_ids = (0, 2, 4, 5, 12, 13, 14, 15)

# Gait Setting
trot_cycle_length = 1.0
trot_duty_ratio = 0.5
path_increment = 0.08
x_front = const(15)
x_back = const(-25)
lift_height = const(15)
kp_height = 0.03
kp_weight = 0.04

# WIFI config
wifissid = 'LeeSophia'
wifipass = 'qq8201602'

# Frame Time
frame_time_ms = 100

# html_dir = 'pydog/control_panel.html'
# html_str = load_panel_html(html_dir)
html_str = """<!DOCTYPE html>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
<html lang="en">
<head><meta charset="UTF-8"><title>瑷琟灵机器人</title></head>
<style type="text/css">
*{padding:0;margin:0;}
button {
color: black;
background:lightgrey;
border: 1px solid #000;
border-radius: 8px;
position: center;
}
a{display: block;width: 100%;height: 100%;line-height: 100px;font-size: 14px;text-align: center;text-decoration:none;color:#000000;}
.content{width: 200px;height: 200px;position: relative;margin:50px auto;left: 0;top:50%;bottom: 0;right:0;
         box-shadow: 0px 0px 550px rgba(255, 255, 255, 0.3) inset,0px 0px 5px #FFFFFF;}
.quartercircle{position:absolute;width: 100px;height: 100px;-webkit-border-radius: 100px 0 0 0;}
.divLeft{top: 25%;left: -10%; transform:rotate(-45deg);background-color: bisque ;}
.divTop{top: -10%;left: 25%; transform:rotate(45deg);background-color: burlywood ;}
.divRight{top: 25%;left: 60%;transform:rotate(135deg);background-color: darkgray ;}
.divBottom{top: 60%;left: 25%;transform:rotate(-135deg);background-color: darkkhaki ;}
.circle{width:50%;height:50%;position: absolute;z-index: 100;top:0%;left:0%;bottom:0;right: 0;margin:auto;border-radius: 100%;
        background-color: #889900;text-align: center;}
.circle span{display: block;width: 100%;height: 100%;line-height: 200px;font-size: 24px;}
.quartercircle a{position: absolute;width: 100%;height: 100%;background: #888888;}
.quartercircle a:hover{background: #8BFF7C;}
.dimSlide { transform: scaleX(2) rotate(0deg); width: 200px;position: relative}
</style>
<script language="javascript">
function dimTimes(evt) {
    let me2;
    const xmlhttp2 = new XMLHttpRequest();
    me2 = evt.target;
    xmlhttp2.open("GET","times=" + me2.value,true); xmlhttp2.send()
}
function dimSeth(evt) {
    let me3;
    const xmlhttp3 = new XMLHttpRequest();
    me3 = evt.target;
    xmlhttp3.open("GET","height=" + me3.value,true); xmlhttp3.send()
}
</script>
<div style="text-align: center;"><h1>NodePyDog 控制器</h1></div><br>
<center><form><span>
<span align="center" style="display:inline-block;padding:5px;border:1px solid #fc0; font-size: 120%;font-weight: bold;">
<p>功能模式</p>
<button name="MODE" value="stand" type="submit" style="width:45px; height:30px; Text-align:center;"> Stand </button>
<button name="MODE" value="move" type="submit" style="width:43px; height:30px;"> Move </button>
<button name="MODE" value="turn" type="submit" style="width:40px; height:30px;"> Turn </button>
<button name="MODE" value="translate" type="submit" style="width:64px; height:30px;"> Translate </button>
<button name="MODE" value="rotate" type="submit" style="width:50px; height:30px;"> Rotate </button>
</span>
<span align="center" style="display:inline-block;padding:5px;border:1px solid #fc0; font-size: 120%;font-weight: bold;">
<p>移动步态</p>
<button name="Gait" value="trot" type="submit" style="width:40px; height:30px; Text-align:center;"> Trot </button>
<button name="Gait" value="gallup" type="submit" style="width:50px; height:30px;"> Gallup </button>
</span>
</span></form><center>
<div class="content" style="">
<form action="/" method="get" accept-charset="utf-8">
<div class="quartercircle divLeft" style="">
<a href="?Key=left" style="background: no-repeat center;transform:rotate(45deg);font-weight: bold;">左转</a>
</div>
<div class="quartercircle divTop" style="">
<a href="?Key=forward" style="background: no-repeat center;transform:rotate(-45deg);font-weight: bold;">前进/前移</a>
</div>
<div class="quartercircle divRight" style="">
<a href="?Key=right" style="background: no-repeat center;transform:rotate(-135deg);font-weight: bold;">右转</a>
</div>
<div class="quartercircle divBottom" style="">
<a href="?Key=backward" style="display:block;position:absolute;z-index:50;background: no-repeat center;transform:rotate(135deg);font-weight: bold;">后退/后移</a>
</div>
<div class="circle" style="background-color: red;font-weight: bold;"><a href="?Key=step_reset">踏步</a></div>
</form>
</div>
<div style="text-align: center;"><h4>- 高度 +</h4></div>
<div style="text-align: center;"><input class= "dimSlide" type="range" id="dimSlide3" min="80" max="110" step="0" value="95" onclick="dimSeth(event)" oninput="dimSeth(event)" ></div>
<div style="text-align: center;"><h4>- 动作次数 +</h4></div>
<div style="text-align: center;"><input class= "dimSlide" type="range" id="dimSlide3" min="1" max="9" step="0" value="5" onclick="dimTimes(event)" oninput="dimTimes(event)" ></div>
<br />
<br />
</body>
</html>"""
