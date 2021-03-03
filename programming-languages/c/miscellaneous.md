# Miscellaneous

* 关于右移位运算的规则
  * 若左操作数类型为 int 或 long，则作算数移位，即使用原数符号位（最高位）填充移位后的空余高位；若左操作数类型为 uint 或 ulong，则作逻辑移位，即使用 0 填充移位后的空余高位。
  * ```text
    int a = int.MinValue;
    int b = a >> 3;
    // Before: 10000000000000000000000000000000
    // After:  11110000000000000000000000000000

    uint c = 0b_1000_0000_0000_0000_0000_0000_0000_0000;
    uint d = c >> 3;
    // Before: 10000000000000000000000000000000
    // After:  00010000000000000000000000000000
    ```
  * ref:  [Bitwise and shift operators - C\# reference \| Microsoft Docs](%20https://docs.microsoft.com/en-us/dotnet/csharp/language-reference/operators/bitwise-and-shift-operators#right-shift-operator-)

