import csv
import pandas as pd
import matplotlib.pyplot as plt

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        df["Time"] -= df["Time"][0]
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None
    
def plot_uav(ax, uav_x, uav_y, uav_z, uav_r):
    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["yaw_vel_uav"].to_numpy(), c='b', label='vel_uav', s=5)

    ax[0].scatter(uav_x["Time"].to_numpy(), uav_x["X_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[1].scatter(uav_y["Time"].to_numpy(), uav_y["Y_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[2].scatter(uav_z["Time"].to_numpy(), uav_z["Z_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)
    ax[3].scatter(uav_r["Time"].to_numpy(), uav_r["yaw_cmd_vel"].to_numpy(), c='r', label='cmd_vel', s=5)

    ax[0].set_ylabel('X')
    ax[1].set_ylabel('Y')
    ax[2].set_ylabel('Z')
    ax[3].set_ylabel('Yaw')

uav_1 = ler_csv("csv/uav_1.csv")
uav_2 = ler_csv("csv/uav_2.csv")
uav_3 = ler_csv("csv/uav_3.csv")
uav_4 = ler_csv("csv/uav_4.csv")

time_x = [20, 120]
time_y = [100, 170]
time_z = [0, 25]
time_r = [160, 250]

uav_x = uav_4[(uav_4["Time"] >= time_x[0]) & (uav_4["Time"] <= time_x[1])]
uav_y = uav_4[(uav_4["Time"] >= time_y[0]) & (uav_4["Time"] <= time_y[1])]
uav_z = uav_4[(uav_4["Time"] >= time_z[0]) & (uav_4["Time"] <= time_z[1])]
uav_r = uav_4[(uav_4["Time"] >= time_r[0]) & (uav_4["Time"] <= time_r[1])]
uav_z["Z_vel_uav"] = 2 * uav_z["Z_vel_uav"]

uav_x.to_csv("csv/uav_x.csv", index=False)
uav_y.to_csv("csv/uav_y.csv", index=False)
uav_z.to_csv("csv/uav_z.csv", index=False)
uav_r.to_csv("csv/uav_r.csv", index=False)

fig, ax = plt.subplots(4, 1, figsize=(15, 9))
plot_uav(ax, uav_x, uav_y, uav_z, uav_r)
plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
