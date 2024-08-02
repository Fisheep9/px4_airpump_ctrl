# PX4 Gimbal Ctrl

使用mavros mount control像控制云台舵机一样进行气泵控制的一个示例（[参考宁哥的云台控制](https://github.com/Fisheep9/px4_gimbal_ctrl)）： 

[气泵开关控制逻辑介绍](https://item.taobao.com/item.htm?spm=a1z0d.6639537/202406.item.d748424345614.64517484ut835Q&id=748424345614&from=cart&skuId=5162411637103&pisk=fmKii3VhJz2bOj2xsHjskZeo8NgK55sfgIEAMiCq865BXA_AMeX2MBMjCGpViIAdOhH1ksL3nLOHiZM1Ms5DMK8tp0nJCdsf0jc-20eAWljvs5S47x5FEOvO08fBCdsfgFJXYbp_oXMvvh72gMSFCTS4QlWqKMWcHirNulzUT66V0sRVu6yFHTeaQ1rVLYSZQsx2zHlNfvEkLU-2xObEJd5eV3HAQNA6I6craHm5S_JNtol8icwG_T-r9P6lSIbevKe8D_JwPI6y_5lR5K8HtOAKOyXHyCtlY3lQmIvWABXeamqR_p9JyNtZkuQlx1bvbIc0oOYJsI1wQWMCvKJFf19KqYODeCxcyCnEQIsFFa8D6XZV6UvetaKL9uIXOERVnBobT9tMeUQMSfqRGZSrUyzPAoEfLxKUlr_Nd9fSI8oWmsewbOHnK50f7965wvD3lJ7Nd9f-Kv4RHNWCF_C.)


## 使用方法

`roslaunch mavros px4.launch`

`roslaunch px4_gimbal_ctrl test.launch`

开启和关闭

`rostopic pub /gimbal_trigger std_msgs/Bool "data: true"`

`rostopic pub /gimbal_trigger std_msgs/Bool "data: false"`

> Status： 测试版 

## 节点流程

使用之前需要启动mavros，启动节点后，通过`/mavros/mount_control/orientation`获取并维持当前气泵状态，然后发送`mavros/mount_control/command`控制指令

> **因为目前云台的运动范围为上下 45 deg，所以在QGC内设置MNT_OFF_PITCH和MNT_OFF_ROLL为 -45 deg，范围变为 0-90 deg**

## 特性

- 将飞控的COM_PREARM_MODE选项打开，具体见PX4文档，这样在上锁模式下仍可以改变云台位置。
  https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html#COM_PREARM_MODE

- 由于PX4 Mount Control的特性，在节点退出后仍然会保持当前位置，只有在飞控重启后进行复位
