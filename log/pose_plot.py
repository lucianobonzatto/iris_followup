import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

def plot_grafico3d(ax, controller, mv):
    iris_pose = ler_csv("log/csv/" + mv + "/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("log/csv/" + mv + "/" + controller + "/magni_pose.csv")
    
    ax.plot(iris_pose["X Position"].to_numpy(), iris_pose["Y Position"].to_numpy(), iris_pose["Z Position"].to_numpy(), c='b', label=f'iris')
    ax.plot(magni_pose["X Position"].to_numpy(), magni_pose["Y Position"].to_numpy(), magni_pose["Z Position"].to_numpy(), c='r', label=f'magni')
    
    ax.set_xlabel('X Position', fontsize=14)  # Defina o tamanho da fonte
    ax.set_ylabel('Y Position', fontsize=14)  # Defina o tamanho da fonte
    ax.set_zlabel('Z Position', fontsize=14)  # Defina o tamanho da fonte

    if(mv == "sqr"):
        ax.set_xticks(np.arange(0, 2.5, 0.5))
        ax.set_yticks(np.arange(-2.5, 0, 0.5))

        ax.set_xlim(0, 2)
        ax.set_ylim(-2, 0)
        ax.set_zlim(0, 2.1)
    else:
        ax.set_xticks(np.arange(0, 5, 1))
        ax.set_yticks(np.arange(-1, 1, 0.5))

        ax.set_xlim(0, 4)
        ax.set_ylim(-1, 1)
        ax.set_zlim(0, 2.1)
        if controller == 'pd':
            ax.set_title("PD Controller", fontsize=16)  # Defina o tamanho da fonte
        elif controller == 'cascade':
            ax.set_title("PD-PI Cascade Controller", fontsize=16)  # Defina o tamanho da fonte
        elif controller == 'paralel':
            ax.set_title("PD-PI Parallel Controller", fontsize=16)  # Defina o tamanho da fonte
    
    ax.legend(fontsize=12)  # Defina o tamanho da fonte para a legenda
    ax.tick_params(axis='both', which='major', labelsize=12)

vel = ["03", "04", "05"]
controller = ['pd', 'cascade', 'paralel']

fig = plt.figure()
ax1 = fig.add_subplot(231, projection='3d')
ax2 = fig.add_subplot(232, projection='3d')
ax3 = fig.add_subplot(233, projection='3d')
ax4 = fig.add_subplot(234, projection='3d')
ax5 = fig.add_subplot(235, projection='3d')
ax6 = fig.add_subplot(236, projection='3d')

plot_grafico3d(ax1, controller[0], 'line')
plot_grafico3d(ax2, controller[1], 'line')
plot_grafico3d(ax3, controller[2], 'line')
plot_grafico3d(ax4, controller[0], 'sqr')
plot_grafico3d(ax5, controller[1], 'sqr')
plot_grafico3d(ax6, controller[2], 'sqr')

plt.subplots_adjust(left=0, bottom=0.05, right=0.97, top=1, wspace=0.17, hspace=0.2)
plt.show()
