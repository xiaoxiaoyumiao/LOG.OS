# Lifecycle

## Game Life

```csharp
// exit game
#if UNITY_EDITOR
	UnityEditor.EditorApplication.isPlaying = false;
#else
	Application.Quit();
#endif

// This function is called at the very last frame
 OnApplicationQuit()
```

ref: [https://answers.unity.com/questions/171688/detect-when-editor-stops-playing.html](https://answers.unity.com/questions/171688/detect-when-editor-stops-playing.html)

## Initialization

`Awake` 和 `Start` 是常用的实现组件初始化的函数。为了保证稳定性，尽量在 `Awake` 中实现一些变量的初始化。细节可参阅官方提供的生命周期。

## Basic Update

更新函数 `update` 和 `fixedupdate` 

* `Update()`：每帧被调用一次，现实中帧率会随时间发生变化，常应用于逻辑处理；也可以通过引用`Time.deltaTime` 使动作和帧间时间挂钩，达到对齐时间的效果
* `FixedUpdate()`：每隔 `Time.fixedDeltaTime` 被调用一次。`Time.fixedDeltaTime` 默认是0.02s，可以通过Edit-&gt;ProjectSettings-&gt;Time来设置。由于和时间相关，常应用于计时、物理动效等
  * 事实上它并不会、也不可能保持稳定的现实时间间隔，而只能在平均意义上保持执行频率与设定的时间间隔一致。尽管如此，在脚本中访问 `Time.fixedDeltaTime` 时仍然能获取到一个定值，这是物理模拟计算的需要。物理模拟并不需要精确的现实时间间隔，只要在一小段时间内物理量能以要求的频率较均匀地被更新，就能取得较好的模拟效果。请参考 [Physics](physics.md) 页面。

参考 \[1\] 中的流图即可。

组件左上角的 checkbox 是 enable / disable 的选项。

{% embed url="https://answers.unity.com/questions/26844/enabledisable-specific-components.html" %}

## Reference

\[1\] [https://docs.unity3d.com/Manual/ExecutionOrder.html](https://docs.unity3d.com/Manual/ExecutionOrder.html)

