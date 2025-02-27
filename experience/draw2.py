import plotly.graph_objects as go
import plotly.offline as pyo
import pandas as pd

# 读取CSV数据
df = pd.read_csv("./trajectory_tracking/flat/data1.csv")

# 创建图表
fig = go.Figure()

# 添加实线数据（红色、绿色、蓝色）
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosX_ref"], mode='lines', name='实线1 (红色)',
    line=dict(color='red', width=2)
))
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosY_ref"], mode='lines', name='实线2 (绿色)',
    line=dict(color='green', width=2)
))
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosZ_ref"], mode='lines', name='实线3 (蓝色)',
    line=dict(color='blue', width=2)
))

# 添加虚线数据（红色、绿色、蓝色）
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosX_cur"], mode='lines', name='虚线1 (红色)',
    line=dict(color='red', width=2, dash='dash')
))
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosY_cur"], mode='lines', name='虚线2 (绿色)',
    line=dict(color='green', width=2, dash='dash')
))
fig.add_trace(go.Scatter(
    x=df["time"], y=df["bodyPosZ_cur"], mode='lines', name='虚线3 (蓝色)',
    line=dict(color='blue', width=2, dash='dash')
))

# 设置图表布局
fig.update_layout(
    title='科研论文曲线图示例',
    xaxis_title='Times(s)',
    yaxis_title='BodyPosition(m)',
    font=dict(family='Arial', size=14, color='black'),  # 字体设置
    legend=dict(x=0.02, y=0.98),  # 图例位置
    plot_bgcolor='white',  # 背景颜色
    showlegend=True,
    xaxis=dict(zeroline=True, ticks='outside', showline=True,showgrid=True, gridcolor='lightgray'),  # X 轴网格
    yaxis=dict(zeroline=True, ticks='outside', showline=True,showgrid=True, gridcolor='lightgray')  # Y 轴网格
)

# 显示图表
# fig.show()
pyo.iplot(fig)

# 保存为图片（可选）
# fig.write_image("research_plot.pdf")  # 保存为高分辨率 PNG