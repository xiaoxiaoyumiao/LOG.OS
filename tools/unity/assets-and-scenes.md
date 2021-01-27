# Assets and Scenes

#### Prefabs

把一个游戏场景中的对象从hierarchy列表中拖回assets栏即可获得一个Prefab。为了动态spawn出游戏对象，可以在Monobehavior的子类中定义一个GameObject的field，并在编辑器中将其赋值为assets中的prefab（这个field的取值只能是prefab或gameobject），然后在update函数中使用Instantiate\(gameobject\)将其实例化：

```csharp
//pseudo code
public class test : MonoBehavior {
    public GameObject gameobject;
    void Update() {
        destroy();//remember to destroy objects that you generated
        if (spawn_condition) {
            GameObject instance = Instantiate(gameobject) as GameObject;
            //do something to instance
        }
    }    
    void destroy() {
        foreach (GameObject ele in list_to_destroy) {
            Destroy(ele);
        }
        list_to_destroy.clear();
    }
}
```

**注意对Prefab的嵌套可能出现子Prefab的关联丢失的问题**，[https://blog.csdn.net/zhenghongzhi6/article/details/84068691](https://blog.csdn.net/zhenghongzhi6/article/details/84068691) ； 似乎能解决这个问题的轮子[http://www.xuanyusong.com/archives/3042](http://www.xuanyusong.com/archives/3042)

**资源加载**

[https://www.cnblogs.com/zhepama/p/4362312.html](https://www.cnblogs.com/zhepama/p/4362312.html)

加载的资源必须在Assets/Resources路径下，查询（Load）时使用项目中该目录下的相对路径。例如Resources/Time.png会被项目加载成Resources目录下的一个资源Time，于是load时传一个”Time“就能索引到这个资源。

```text
foreach (string ele in names)
{
    Object pref = Resources.Load(string.Format(directory, ele), typeof(Sprite));
    Sprite tmp = GameObject.Instantiate(pref) as Sprite;
    sprites.Add(ele, tmp);
}
```

正规大型游戏更倾向采用在Runtime加载AssetBundle的方式，目前还没有试验过，待补完。

