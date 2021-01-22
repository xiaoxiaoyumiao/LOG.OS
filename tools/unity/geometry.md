# Geometry

**Sprite**

脚本中修改sprite的图片来源，只需获取SpriteRenderer组件，修改其sprite域即可。

sprite 资源的 pivot 可以自定义，通过这样来调整图片资源中心位置的偏移

获取某个物体的 sprite 尺寸：

```text
float width = object.GetComponent<Renderer>().bounds.size.x;
```

**直线绘制**

似乎至少有GL和LineRenderer两种搞法，具体待实验

真是要了老命了……

* 一个游戏对象只能用一个lineRenderer，一个lineRenderer只能画一条线（当然可以是长的折线段或者近似出来的曲线）。
* 绘制时使用的是世界坐标系。如果绘制坐标和物体相关，直接使用物体的transform坐标运算即可。
* 因为linerenderer画出来的是3D对象，会因为未知原因导致被背景之类的遮挡。把背景的z坐标调大一些（比如10，反正一个恰当的正数），就可以看到画出来的线了。
* 2D环境下需要恰当选择线的材质，一个可以运作的着色器是选择shader: Unlit - Color，然后在Tint中配色。
  * update：不需要自建材质，应当使用unity自带的默认材质default-line，此时对线的color设置才可以生效。点击inspector中右侧小圆点，从弹出的列表里选择。

一段可用的代码：（材质和颜色都在inspector里配置，此外颜色似乎没发挥作用）

```text
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

