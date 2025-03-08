import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV数据
df = pd.read_csv("./trajectory_planning/full_plan/slow/data2.csv")

# # 确保 'time' 列被解析为 datetime 类型
# df['time'] = pd.to_numeric(df['time'])

# 选择时间范围
# start_date = 0.5
# end_date = 10.
# mask = (df['time'] >= start_date) & (df['time'] <= end_date)
# filtered_df = df.loc[mask]
filtered_df = df

# 创建图表
plt.figure(figsize=(8, 6))

# plt.plot(filtered_df["time"], filtered_df["Pbx"], label='Xcur', color='red',   linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Pby"], label='Ycur', color='green', linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Pbz"], label='Zcur', color='blue',  linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Pcx"], label='Xref', linestyle='dashdot', color='red',   linewidth=2, alpha=0.7)
# plt.plot(filtered_df["time"], filtered_df["Pcy"], label='Yref', linestyle='dashdot', color='green', linewidth=2, alpha=0.7)
# plt.plot(filtered_df["time"], filtered_df["Pcz"], label='Zref', linestyle='dashdot', color='blue',  linewidth=2, alpha=0.7)
# plt.plot(filtered_df["time"], filtered_df["Prx"], label='Xcmd', linestyle='dashed', color='red',   linewidth=2, alpha=0.6)
# plt.plot(filtered_df["time"], filtered_df["Pry"], label='Ycmd', linestyle='dashed', color='green', linewidth=2, alpha=0.6)
# plt.plot(filtered_df["time"], filtered_df["Prz"], label='Zcmd', linestyle='dashed', color='blue',  linewidth=2, alpha=0.6)

# plt.plot(filtered_df["time"], filtered_df["Rollc"], label='Roll_cur', color='red',   linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Pitchc"], label='Pitch_cur', color='green', linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Yawc"], label='Yaw_cur', color='blue',  linewidth=2, alpha=0.9)
# plt.plot(filtered_df["time"], filtered_df["Rollt"], label='Roll_cmd', linestyle='dashed', color='red',   linewidth=2, alpha=0.6)
# plt.plot(filtered_df["time"], filtered_df["Pitcht"], label='Pitch_cmd', linestyle='dashed', color='green', linewidth=2, alpha=0.6)
# plt.plot(filtered_df["time"], filtered_df["Yawt"], label='Yaw_cmd', linestyle='dashed', color='blue',  linewidth=2, alpha=0.6)

# plt.plot(filtered_df["time"], filtered_df["Forcex"], label='Force_x', color='red', linewidth=2, alpha=0.8)
# plt.plot(filtered_df["time"], filtered_df["Forcey"], label='Force_y', color='green', linewidth=2, alpha=0.8)

# plt.plot(filtered_df["time"], filtered_df["ux"], label='ux', color='red', linewidth=2, alpha=0.8)
# plt.plot(filtered_df["time"], filtered_df["uy"], label='uy', color='green', linewidth=2, alpha=0.8)

plt.plot(filtered_df["time"], filtered_df["Pf0rx"], label='LFref', linestyle='dashed', color='cyan', linewidth=2, alpha=0.6)
plt.plot(filtered_df["time"], filtered_df["Pf0cx"], label='LFcur', color='cyan', linewidth=2, alpha=0.9)
plt.plot(filtered_df["time"], filtered_df["Pf1rx"], label='RFref', linestyle='dashed', color='magenta', linewidth=2, alpha=0.6)
plt.plot(filtered_df["time"], filtered_df["Pf1cx"], label='RFcur', color='magenta', linewidth=2, alpha=0.9)
plt.plot(filtered_df["time"], filtered_df["Pf2rx"], label='LBref', linestyle='dashed', color='yellow', linewidth=2, alpha=0.6)
plt.plot(filtered_df["time"], filtered_df["Pf2cx"], label='LBcur', color='yellow', linewidth=2, alpha=0.9)
plt.plot(filtered_df["time"], filtered_df["Pf3rx"], label='RBref', linestyle='dashed', color='black', linewidth=2, alpha=0.6)
plt.plot(filtered_df["time"], filtered_df["Pf3cx"], label='RBcur', color='black', linewidth=2, alpha=0.9)

# 设置标题和坐标轴标签
plt.title('balance constrain')
plt.xlabel('Times(s)')
plt.ylabel('u')

# 显示网格
plt.grid()

# 显示图例
plt.legend()

# 保存图片
plt.savefig("./trajectory_planning/full_plan/slow/foot.png")

# 显示图表
plt.show()