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

def plot_grafico3d(controller, mv):
    iris_pose = ler_csv("log/csv/" + mv + "/" + controller + "/iris_pose.csv")
    magni_pose = ler_csv("log/csv/" + mv + "/" + controller + "/magni_pose.csv")

    iris_pose["Z Position"] = iris_pose["Z Position"] + 0.25
    iris_pose.to_csv("teste.csv")






controller = ['pd', 'cascade', 'paralel']

# plot_grafico3d(ax1, controller[0], 'line')
# plot_grafico3d(ax2, controller[1], 'line')
plot_grafico3d(controller[2], 'line')
# plot_grafico3d(ax4, controller[0], 'sqr')
# plot_grafico3d(ax5, controller[1], 'sqr')
# plot_grafico3d(controller[2], 'sqr')

# plt.subplots_adjust(left=0, bottom=0.05, right=0.97, top=1, wspace=0.17, hspace=0.2)
# plt.show()