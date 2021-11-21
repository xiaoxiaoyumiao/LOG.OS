# Miscellaneous

* 广播机制xxxmessage
  * //向自身和子类发送消息 gameObject.BroadcastMessage\("ReceiveBroadcastMessage", "A0-BroadcastMessage\(\)"\);
  * //给自己发送消息 gameObject.SendMessage\("ReceiveSendMessage", "A0-SendMessage\(\)"\);
  * //向自身和父类发送消息 gameObject.SendMessageUpwards\("ReceiveSendMessageUpwards", "A0-ReceiveSendMessageUpwards\(\)"\);
* 关于active
  * 这是游戏对象具有的一个属性，inactive的对象和子对象应该是不会被渲染和更新的，也会影响到查找游戏对象的结果
  * 可以通过调用`gameObject.SetActive(bool)`来设置
* 2D中transform.lookAt和transform.forward并不造成平面上的旋转看齐而是直接消失，推测以z轴看齐导致图像与画面垂直。看齐应对transform的具体轴做修改，如transform.right = vector.
* 关于贴图缺失
  * meta 文件包含了对资源文件的 GUID 描述，而 inspector 中的贴图信息都是基于 GUID 绑定的。每当 unity 为一个新的资源文件创建 meta 文件，就会分配一个新的 GUID。所以当发现 inspector 提示贴图缺失时，把缺失的贴图放入相同的目录位置并不能解决问题，必须导入原来的 meta 文件才是科学的。
* 关于错误报告
  * 如果 Unity 因为不明原因崩溃了，通过其留下的 Log 文件往往可以获得一些调试的线索。Log 文件的路径可参考官方文档。
  * ref：[https://docs.unity3d.com/Manual/LogFiles.html](https://docs.unity3d.com/Manual/LogFiles.html)
* 一个带有member的class被绑定到一个游戏中的实体（拖动放置到实体）时，实体会多出一个与class名字一致的属性组（在unity里似乎叫做component），且具有与member一致的子属性。如果script更新，unity中的属性也会随之更新。
* 碰撞盒是一类常用的component，包括box collider 2d和rigidbody 2d等

