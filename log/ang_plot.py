import pandas as pd
import matplotlib.pyplot as plt

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

def plot_grafico2d(ax, controller):
    iris_pose = ler_csv("log/csv/sqr/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("log/csv/sqr/" + controller + "/magni_pose.csv")

    iris_pose["Time"] -= iris_pose["Time"][0]
    magni_pose["Time"] -= magni_pose["Time"][0]

    if controller == "pd":
        i = 0
        while i < len(iris_pose["Time"]):
            if((iris_pose["Time"][i] > 20) and (iris_pose["Time"][i] < 30.5)):
                if(iris_pose["yaw"][i] > 0):
                    iris_pose["yaw"][i] = -iris_pose["yaw"][i]
            i = i + 1

        i = 0
        while i < len(magni_pose["Time"]):
            if((magni_pose["Time"][i] > 20) and (magni_pose["Time"][i] < 28)):
                if(magni_pose["yaw"][i] > 0):
                    magni_pose["yaw"][i] = -magni_pose["yaw"][i]
            i = i + 1

    if controller == "cascade":
        i = 0
        while i < len(iris_pose["Time"]):
            if((iris_pose["Time"][i] > 20) and (iris_pose["Time"][i] < 29.55)):
                if(iris_pose["yaw"][i] > 0):
                    iris_pose["yaw"][i] = -iris_pose["yaw"][i]
            i = i + 1

        i = 0
        while i < len(magni_pose["Time"]):
            if((magni_pose["Time"][i] > 20) and (magni_pose["Time"][i] <= 29.15)):
                if(magni_pose["yaw"][i] > 0):
                    magni_pose["yaw"][i] = -magni_pose["yaw"][i]
            i = i + 1

    ax[0].plot(iris_pose["Time"].to_numpy(), iris_pose["X Position"].to_numpy(), c='b', label=f'iris')
    ax[0].plot(magni_pose["Time"].to_numpy(), magni_pose["X Position"].to_numpy(), c='r', label=f'magni')
    
    ax[1].plot(iris_pose["Time"].to_numpy(), iris_pose["Y Position"].to_numpy(), c='b', label=f'iris')
    ax[1].plot(magni_pose["Time"].to_numpy(), magni_pose["Y Position"].to_numpy(), c='r', label=f'magni')

    ax[2].plot(iris_pose["Time"].to_numpy(), iris_pose["yaw"].to_numpy(), c='b', label=f'iris')
    ax[2].plot(magni_pose["Time"].to_numpy(), magni_pose["yaw"].to_numpy(), c='r', label=f'magni')
    
    for a in ax.flat:
        a.set_xlabel('Time')
        a.legend()

    ax[0].set_ylabel('X position (m)')
    ax[1].set_ylabel('Y position (m)')
    ax[2].set_ylabel('Yaw position (rad)')

    if controller == 'pd':
        ax[0].set_title("PD Controller")
    elif controller == 'cascade':
        ax[0].set_title("PD-PI Cascade Controller")
    elif controller == 'paralel':
        ax[0].set_title("PD-PI Parallel Controller")

controller = ['pd', 'cascade', 'paralel']

fig, ax = plt.subplots(3, len(controller), figsize=(15, 9), sharex='col', sharey='row')

for i, ctrl in enumerate(controller):
    plot_grafico2d(ax[:, i], ctrl)

plt.subplots_adjust(left=0.05, bottom=0.1, right=0.97, top=0.95, wspace=0.15, hspace=0.4)
plt.show()
