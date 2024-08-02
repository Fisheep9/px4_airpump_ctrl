# PX4 Gimbal Ctrl

使用mavros mount control像控制云台舵机一样进行气泵控制的一个示例（参考宁哥的云台控制）： 


> Status： 测试版
p
## 节点流程

使用之前需要启动mavros，启动节点后，通过`/mavros/mount_control/orientation`获取并维持当前气泵状态

> **因为目前云台的运动范围为上下45 deg，所以在QGC内设置offset，范围变为0-90deg**
>
> **![image-20240802180408491](/home/hilab/.config/Typora/typora-user-images/image-20240802180408491.png)**

## 特性

- 可以将飞控的Prearm选项打开，具体见PX4文档，这样在上锁模式下仍可以改变云台位置。
  https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html#COM_PREARM_MODE

  ![image-20240802180647130](/home/hilab/.config/Typora/typora-user-images/image-20240802180647130.png)

- 由于PX4 Mount Control的特性，在节点退出后仍然会保持当前位置，只有在飞控重启后进行复位