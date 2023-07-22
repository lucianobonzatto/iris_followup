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
    iris_pose = ler_csv("csv/sqr/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("csv/sqr/" + controller + "/magni_pose.csv")
    
    ax[0].plot(iris_pose["Time"].to_numpy(), iris_pose["X Position"].to_numpy(), c='b', label=f'iris_pose')
    ax[0].plot(magni_pose["Time"].to_numpy(), magni_pose["X Position"].to_numpy(), c='r', label=f'magni_pose')
    
    ax[1].plot(iris_pose["Time"].to_numpy(), iris_pose["Y Position"].to_numpy(), c='b', label=f'iris_pose')
    ax[1].plot(magni_pose["Time"].to_numpy(), magni_pose["Y Position"].to_numpy(), c='r', label=f'magni_pose')

    ax[2].plot(iris_pose["Time"].to_numpy(), iris_pose["yaw"].to_numpy(), c='b', label=f'iris_pose')
    ax[2].plot(magni_pose["Time"].to_numpy(), magni_pose["yaw"].to_numpy(), c='r', label=f'magni_pose')
    
    # ax[0].set_xlabel('Index')
    # ax[0].set_ylabel('Yaw')
    # ax[0].set_ylim(-3.14, 3.14)
    ax[0].legend()

    # ax[1].set_xlabel('Index')
    # ax[1].set_ylabel('Yaw')
    # ax[1].set_ylim(-3.14, 3.14)
    ax[1].legend()

    ax[2].set_xlabel('Index')
    ax[2].set_ylabel('Yaw')
    ax[2].set_ylim(-3.14, 3.14)
    ax[2].legend()

controller = ['pd', 'cascade', 'paralel']

fig, ax = plt.subplots(3, 3, figsize=(6, 15))

plot_grafico2d(ax[0], controller[0])
plot_grafico2d(ax[1], controller[1])
plot_grafico2d(ax[2], controller[2])

# plt.subplots_adjust(left=0, bottom=0.05, right=0.97, top=1, hspace=0.4)
plt.show()
