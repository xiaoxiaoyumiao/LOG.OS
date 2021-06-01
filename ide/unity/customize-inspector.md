# Customize Inspector

## 自定义Inspector

{% embed url="https://blog.csdn.net/qq\_33337811/article/details/62042218" %}



## 序列化

为了让component的field能够在inspector中便捷地被访问和修改，需要在类前标注这是一个可序列化的类，并对不需要序列化的值标注非序列化。

```text
[System.Serializable]
public class BlockTypeParameter
{
    public int someProperty;
    [System.NonSerialized]
    private static BlockTypeParameter mFactory;
    ...
}
```

