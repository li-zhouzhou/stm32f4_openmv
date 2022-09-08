# stm32f4_openmv
基于stm32非接触式物体尺寸形态测量仪代码(本科电赛)
12V直流稳压电源通过5V和3.3.V稳压模块分别向单片机和OpenMV供电；目标物体的水平距离通过激光测距模块读取，并转化为距离数字量传回单片机中；目标物体的几何形状以及坐标通过OpenMV的颜色识别读取，并转化为位置数字量传回单片机中；单片机将激光测距模块读取的距离数据处理后传给OpenMV，OpenMV经比例计算后返回物体边长的数字量；最后单片机将目标物体的形状、边长和距离数据传给OLED显示；进入自动寻找目标时，单片机根据OpenMV传回的坐标数据控制云台舵机转动角度；当测量结果稳定后启动声光提示，表示当前测量任务完成；单片机的电平信号可控制由发光二极管和有源蜂鸣器组成的声光提示部分。