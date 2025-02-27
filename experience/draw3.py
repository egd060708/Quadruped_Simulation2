import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV数据
df = pd.read_csv("./trajectory_tracking/flat/data1.csv")

# # 确保 'time' 列被解析为 datetime 类型
# df['time'] = pd.to_numeric(df['time'])

# 选择时间范围
start_date = 0.5
end_date = 10.
mask = (df['time'] >= start_date) & (df['time'] <= end_date)
filtered_df = df.loc[mask]

# 创建图表
plt.figure(figsize=(10, 6))

# 添加三组实线数据
plt.plot(filtered_df["time"], filtered_df["bodyPosX_cur"], label='Group 1 (Solid)', color='red',   linewidth=2, alpha=0.7)
plt.plot(filtered_df["time"], filtered_df["bodyPosY_cur"], label='Group 2 (Solid)', color='green', linewidth=2, alpha=0.7)
plt.plot(filtered_df["time"], filtered_df["bodyPosZ_cur"], label='Group 3 (Solid)', color='blue',  linewidth=2, alpha=0.7)

# 添加三组虚线数据
plt.plot(filtered_df["time"], filtered_df["bodyPosX_ref"], label='Group 4 (Dashed)', linestyle='dashed', color='red',   linewidth=2, alpha=0.5)
plt.plot(filtered_df["time"], filtered_df["bodyPosY_ref"], label='Group 5 (Dashed)', linestyle='dashed', color='green', linewidth=2, alpha=0.5)
plt.plot(filtered_df["time"], filtered_df["bodyPosZ_ref"], label='Group 6 (Dashed)', linestyle='dashed', color='blue',  linewidth=2, alpha=0.5)

# 设置标题和坐标轴标签
plt.title('research')
plt.xlabel('Times(s)')
plt.ylabel('Pos(m)')

# 显示网格
plt.grid()

# 显示图例
plt.legend()

# 显示图表
plt.show()