# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import sys

# 从命令行获取操作符和两个数值
operator = sys.argv[1]
num1 = float(sys.argv[2])
num2 = float(sys.argv[3])

result = 0
# 根据操作符进行运算
if operator == "+":
    result = num1 + num2
elif operator == "-":
    result = num1 - num2
elif operator == "*":
    result = num1 * num2
elif operator == "/":
    result = num1 / num2
else:
    print("Invalid operator")

# 返回计算结果
print(result)


