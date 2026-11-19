# Serial Tool (Windows)

针对本工程（STM32F030 咖啡机固件）调试用的 Windows 串口工具。

## 功能

1. **串口选择与配置**：自动列出所有已识别的 COM 口；可选波特率、数据位、校验位、停止位。
2. **实时接收显示**：滚动文本区域实时打印串口数据，支持时间戳、自动滚屏、保存日志、发送数据。
3. **关键字曲线绘制**：自动识别 `pressure: <数值>` 和 `temp: <数值>` 两类输出，
   提取数值并绘制在同一坐标系上：
   - **pressure** → 蓝色
   - **temp**     → 红色

   匹配规则（不区分大小写，分隔符可为 `:` 或 `=`）：
   ```
   pressure: 12.34
   PRESSURE = 7
   temp:25.6
   ```

## 运行

### 方式一：双击启动（推荐）
直接双击 `run.bat`，首次运行会自动安装依赖。

### 方式二：手动
```cmd
pip install -r requirements.txt
python serial_tool.py
```

需要 Python 3.8+。

## 与本工程的配合

[project/main.c](../../project/main.c) 中生产模式主循环每 500 ms 通过 USART1
（PA9/PA10，115200 8N1）打印温度/压力等信息。要让本工具捕获曲线，固件
端 `printf` 行格式只要包含：

```c
printf("temp:%.2f\r\n", g_current_temp);
printf("pressure:%.2f\r\n", g_pressure_bar);
```

即可。同一行出现多个关键字也支持，例如：

```c
printf("temp:%.1f pressure:%.2f\r\n", t, p);
```
