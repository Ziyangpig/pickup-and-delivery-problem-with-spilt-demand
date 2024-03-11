import pandas as pd

order = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-货盘.csv", encoding='utf-8')
vehicle = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-运力.csv", encoding='gbk')
port_info = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口信息表.csv", encoding='utf-8')
port_restr = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口限制表.csv", encoding='utf-8')
port_draft = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港口吃水限制.csv", encoding='utf-8')
travel = pd.read_csv(r"D:\university\实习\SRIBD\中远数据\month1\输入-港间航行表.csv", encoding='utf-8')

travel = travel.drop_duplicates(subset=['开始港口', '结束港口'], keep='first')

order.to_json('order.json',orient='records')
vehicle.to_json('vehicle.json',orient='records')
port_info.to_json('port_info.json',orient='records')
port_restr.to_json('port_restr.json',orient='records')
port_draft.to_json('port_draft.json',orient='records')
travel.to_json('travel.json',orient='records')

order = pd.read_json("order.json", orient="records")
print(order)



