import pandas as pd
import plotly.express as px

# 读取CSV数据
df = pd.read_csv("./trajectory_tracking/flat/data1.csv")

# 使用Plotly绘制交互式曲线图
fig = px.line(
    df,
    x="time",
    y=["bodyPosX_ref","bodyPosX_cur","bodyPosY_ref", "bodyPosY_cur","bodyPosZ_ref", "bodyPosZ_cur"],  # 多列数据绘制在同一图中
    title="Experiment Data Visualization",
    labels={"value": "bodyPos",
            "time": "Time (s)"},
    template="simple_white"  # 主题风格（可选）
)

# 自定义布局（字体、坐标轴等）
fig.update_layout(
    font_family="Arial",
    xaxis_title_font_size=14,
    yaxis_title_font_size=14,
    legend_title_text="Data Series"
)

# 显示图表（Jupyter中直接显示，或保存为HTML/图片）
fig.show()

# 保存为独立HTML文件
# fig.write_html("plot.html")

# 保存为静态图片（需安装kaleido）
# fig.write_image("plot.png", engine="kaleido")