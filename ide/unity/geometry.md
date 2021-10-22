# Geometry

## **Sprite**

脚本中修改sprite的图片来源，只需获取SpriteRenderer组件，修改其sprite域即可。

sprite 资源的 pivot 可以自定义，通过这样来调整图片资源中心位置的偏移

获取某个物体的 sprite 尺寸：

```csharp
float width = object.GetComponent<Renderer>().bounds.size.x;
```

## **直线绘制**

至少有 GL 和 `LineRenderer` 两种搞法。

使用 `LineRenderer` 时的注意事项：

* 一个游戏对象只能用一个 `lineRenderer` ，一个 `lineRenderer` 只能画一条线（当然可以是长的折线段或者近似出来的曲线）。
* 绘制时使用的是世界坐标系。如果绘制坐标和物体相关，直接使用物体的 transform 坐标运算即可。
* 因为 `lineRenderer` 画出来的是 3D 对象，会因为未知原因导致被背景之类的遮挡。把背景的z坐标调大一些（比如10，反正一个恰当的正数），就可以看到画出来的线了。
* 2D环境下需要恰当选择线的材质，应当使用unity自带的默认材质default-line，此时对线的color设置才可以生效。点击inspector中右侧小圆点，从弹出的列表里选择。

一段可用的代码：（材质和颜色都在inspector里配置，此外颜色似乎没发挥作用）

```csharp
public void PaintLine(Vector3 start, Vector3 end)
    {
        GameObject line = new GameObject();
        lines.Add(line); // 统一管理动态生成的物体，以便销毁
        line.transform.parent = transform;
        myRenderer = line.AddComponent<LineRenderer>();
​
        myRenderer.material = material;
        myRenderer.startColor = lineColor;
        myRenderer.endColor = lineColor;
​
        myRenderer.startWidth = lineSize;
        myRenderer.endWidth = lineSize;
​
        myRenderer.positionCount = 2;
        myRenderer.SetPosition(0, start);
        myRenderer.SetPosition(1, end); 
        // renderer根据一个顶点构成的序列来绘制所需的线条，这里setPosition的第二个参数就代表要设置的顶点坐标，第一个参数代表这个顶点在序列中的索引位置，从0开始
​
    }
```

## **几何变换**

```csharp
gameObject.transform.position = new Vector...
```

`transform.localScale`可以用来处理拉伸和缩放，缩放方式和对象的锚点有关（可以在inspector里查看）。

这些属性的改变大概都是会影响到子对象的。不过在动态创建的时候似乎有办法避免：

> 这个问题就涉及到instantiate这个方法的使用了。
>
> 正常的一个父物体A=（0.5,0.5,0.5） 子物体B（1,1,1）
>
> 如果我是使用instantiate\(B,A\) //实例化子物体B，同时指定B的父物体是A。
>
> 这时子物体B的localscale仍是（1,1,1），但是我发现B物体的absscale变了，现在是（0.5,0.5,0.5,）
>
> 但是我将实例化物体这一过程分成两部
>
> 首先Gameobject go = Instantiate（B）；
>
> 然后go.transform.parent = A;
>
> 这样子物体B的localscale会变成（2,2,2） ， 这样子就没有改变他的absscale。

矩形变换 `RectTransform` 组件中的宽和高可以通过 `rectTransform.sizeDelta` 2D 向量来设置。

ref: [https://blog.csdn.net/xiaomuzi0802/article/details/42214067](https://blog.csdn.net/xiaomuzi0802/article/details/42214067)



{% embed url="https://blog.csdn.net/u010718707/article/details/42111567" %}

## **NavMesh 寻路**

```csharp
UnityEngine.AI.NavMeshAgent
private NavMeshAgent agent; // 用于获得一个寻路AI实例agent
// in Start()
agent = GetComponent<NavMeshAgent>(); 
// ...
agent.enabled = true; // 用于启用agent
agent.SetDestination(point); // 用于设定本游戏对象的寻路目标
agent.remainingDistance; // 用于获取剩余距离
```

## **TileMap**

TileMap 是一种处理栅格布局（如各种经典2D横版过关冒险游戏）地图的组件，因其便利已经成为了 Unity 项目的默认插件之一。其基本使用方法为：

1. 使用 Sprite Editor 将 Sprite 切割为瓦片（Tile）；
2. 将 Tile 导入瓦片调色板（Tile Palette）；
3. 使用调色板在 TileMap 对象中绘制地图。

瓦片是构成 TileMap 地图的基本单元。创建 TileMap 对象并选中时可以看到网格，每个网格就可以放置一个瓦片。用于制作瓦片的 Sprite 一般包含了多种瓦片（比如一种地形的四个边、四个角和内部的图样），在 Sprite 的 Inspector 中可以将其 Sprite Mode 设置为 Multiple 的，表明其包含了复数个 Sprite；通过 Sprite Editor （通过 Inspector 中的按钮可以打开）则可以将其切割。Sprite Editor 中可以设置不同的切割模式，如 Automatic 会根据非透明区域等自动切割，Grid by Cell Size 则会按照一定的尺寸（Pixel Size）切割。通过 Pixels Per Unit 和切割模式的设置可以得到合适大小的瓦片。

通过 Window - 2D - Tile Palette 可以调出瓦片调色板。通过 Create New Palette 创建一个新的调色板（这是一种 asset），将 Sprite 直接拖入调色板窗口即可得到可用于绘制地图的瓦片。上方的工具栏中，刷子用于绘制，橡皮擦等则可以擦除。按下上方的 Edit 进入编辑模式，即可以对调色板进行修改（似乎需要先选中瓦片然后进入编辑模式）。选中多个瓦片就可以一次绘制更大的区域。

绘制时瓦片旋转相关快捷键：（ref：[https://docs.unity3d.com/Manual/Tilemap-Painting.html](https://docs.unity3d.com/Manual/Tilemap-Painting.html)）

| Shortcut Key | Function |
| :--- | :--- |
| \[ | Rotate the active Brush clockwise. |
| \] | Rotate the active Brush anti-clockwise. |
| Shift + \[ | Flip the active Brush along the x-axis. |
| Shift + \] | Flip the active Brush along the y-axis. |
| - \(Available when **Can Change Z Position** is enabled\) | Increase the z-position of the active Brush. This is only available when **Can Change Z Position** is enabled for the active Brush. |
| = \(Available when **Can Change Z Position** is enabled\) | Decrease the z-position of the active Brush. This is only available when **Can Change Z Position** is enabled for the active Brush. |

{% embed url="https://blog.csdn.net/qq\_39162826/article/details/112144150" %}

{% embed url="https://blog.csdn.net/seemeno/article/details/93136806" %}

在代码中可以通过SetTile\(Vector3Int, Tile\)来往固定位置放瓦片。这里的固定位置是TileMap自己的一套坐标系，目前还不清楚是怎么计算的。2D情况下Vector3Int的z轴（第三维）取0。从Sprite加载Tile如下：

```csharp
Tile tile = ScriptableObject.CreateInstance<Tile>();
Sprite tmp = Utility.GetSprite(SpriteType.RANDOM_ROAD);
tile.sprite = tmp;
```

如何给unity的tileMap里的cell添加特定的属性。

做法就是写一个类，继承至TileBase，然后这个类你可以写任何你可以自定义的函数。

但值得注意的是，这里创建的是一个模板，我们依然无法让2个一样模板的tile状态不同。（？）

{% embed url="https://www.cnblogs.com/beatless/p/11623709.html" %}

有一个官方开发的2d-extras仓库用来扩展 tilemap 功能，例如规则瓦片、随机瓦片、动画瓦片

当 tilemap 显示粉红色的时候，检查一下 tilemap 组件的材质属性。

## **血条实现**

有 FillAmount 和 localScale 两种手段，但前者会导致反复重绘，后者性能更优

## **对象的布局 Layout**

Inspector的rect transform可以看到有个方形图案指示的锚点位置设置，默认是锚点居中的，如果想填充父对象，可以选择在两个方向上stretch（最右下角的），并在右边的参数中把left right top bottom都改成0（这里和CSS很像）。高级布局需要借助layout，待补充

