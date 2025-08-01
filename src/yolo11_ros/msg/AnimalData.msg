# AnimalData.msg - 动物数据消息定义

# 五种动物对应的uint8变量
uint8 peacock    # 孔雀
uint8 wolf       # 狼  
uint8 monkey     # 猴子
uint8 tiger      # 老虎
uint8 elephant   # 大象

# 二维坐标数组
geometry_msgs/Point[] coordinates

# 消息头信息（可选）
std_msgs/Header header