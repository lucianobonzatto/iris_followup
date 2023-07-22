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

def plot_grafico3d(ax, controller, vel):
    # iris_pose = ler_csv("log/csv/" + controller + vel + "/iris_pose.csv")
    # magni_pose = ler_csv("log/csv/" + controller + vel + "/magni_pose.csv")

    iris_pose = ler_csv("csv/sqr/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("csv/sqr/" + controller + "/magni_pose.csv")

    # iris_pose = ler_csv("csv/sqr/pd/iris_pose.csv")
    # magni_pose = ler_csv("csv/sqr/pd/magni_pose.csv")
    
    ax.plot(iris_pose["X Position"].to_numpy(), iris_pose["Y Position"].to_numpy(), iris_pose["Z Position"].to_numpy(), c='b', label=f'iris_pose')
    ax.plot(magni_pose["X Position"].to_numpy(), magni_pose["Y Position"].to_numpy(), magni_pose["Z Position"].to_numpy(), c='r', label=f'magni_pose')
    
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')

    ax.set_xlim(0, 2)
    ax.set_ylim(-2, 0)
    ax.set_zlim(0, 2.1)
    
    # ax.legend()

vel = ["03", "04", "05"]
controller = ['pd', 'cascate', 'paralel']

fig = plt.figure()
ax1 = fig.add_subplot(311, projection='3d')
ax2 = fig.add_subplot(312, projection='3d')
ax3 = fig.add_subplot(313, projection='3d')

plot_grafico3d(ax1, controller[0], vel[0])
plot_grafico3d(ax2, controller[1], vel[0])
plot_grafico3d(ax3, controller[2], vel[0])

################################
# ax1 = fig.add_subplot(331, projection='3d')
# ax2 = fig.add_subplot(332, projection='3d')
# ax3 = fig.add_subplot(333, projection='3d')

# ax4 = fig.add_subplot(334, projection='3d')
# ax5 = fig.add_subplot(335, projection='3d')
# ax6 = fig.add_subplot(336, projection='3d')

# ax7 = fig.add_subplot(337, projection='3d')
# ax8 = fig.add_subplot(338, projection='3d')
# ax9 = fig.add_subplot(339, projection='3d')

# plot_grafico3d(ax1, controller[0], vel[0])
# plot_grafico3d(ax2, controller[0], vel[1])
# plot_grafico3d(ax3, controller[0], vel[2])

# plot_grafico3d(ax4, controller[1], vel[0])
# plot_grafico3d(ax5, controller[1], vel[1])
# plot_grafico3d(ax6, controller[1], vel[2])

# plot_grafico3d(ax7, controller[2], vel[0])
# plot_grafico3d(ax8, controller[2], vel[1])
# plot_grafico3d(ax9, controller[2], vel[2])
################################

plt.subplots_adjust(left=0, bottom=0.05, right=0.97, top=1, wspace=0.17, hspace=0.2)
plt.show()