import pandas as pd
import matplotlib.pyplot as plt

csv_file = 'ESP32_LIGHTSLEEP_1MIN.csv' 

df = pd.read_csv(csv_file)

df['datetime'] = pd.to_datetime(df['datetime'])

df = df.sort_values('datetime')

mean_consumption = df['power'].mean()
max_consumption = df['power'].max()
max_consumption_hour = df.loc[df['power'].idxmax(), 'datetime']

print(f"Mean consumption: {mean_consumption:.2f} W")
print(f"Max consumption: {max_consumption:.2f} W ({max_consumption_hour})")

plt.figure(figsize=(12, 6))
plt.plot(df['datetime'], df['power'], label='Consumption (W)', color='blue')
plt.axhline(mean_consumption, color='orange', linestyle='--', label=f'Mean: {mean_consumption:.2f} W')
plt.axvline(max_consumption_hour, color='red', linestyle='--', label=f'Max: {max_consumption:.2f} W')

plt.title('Energy consumption')
plt.xlabel('Date')
plt.ylabel('Power (W)')
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
