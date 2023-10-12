import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import interp1d


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

mvs = ['line', 'sqr']
controllers = ['pd', 'cascade', 'paralel']

print(  "\t\t",
        "mean_Z\t",
        "std_Z\t",
        "mean_x\t",
        "std_x\t",
        "mean_y\t",
        "std_y\t",
        "mean_r\t",
        "std_r\t",
        )

for mv in mvs:
    for controller in controllers:
        iris_pose = ler_csv("log/csv/" + mv + "/" + controller + "/iris_pose.csv")
        magni_pose = ler_csv("log/csv/" + mv + "/" + controller + "/magni_pose.csv")

        iris_Z = iris_pose["Z Position"].to_numpy()
        magni_Z = magni_pose["Z Position"].to_numpy()
        expect_Z = magni_Z.mean() + 1.5
        erro_medio_Z = np.mean(np.abs(iris_Z - expect_Z))
        desvio_padrao_Z = np.std(iris_Z - expect_Z)


        interpolated_magni_pose_x = interp1d(magni_pose['Time'], magni_pose['X Position'])(iris_pose['Time'])
        diferenca_x = iris_pose['X Position'] - interpolated_magni_pose_x
        desvio_padrao_x = diferenca_x.std()
        erro_medio_x = diferenca_x.abs().mean()


        interpolated_magni_pose_y = interp1d(magni_pose['Time'], magni_pose['Y Position'])(iris_pose['Time'])
        diferenca_y = iris_pose['Y Position'] - interpolated_magni_pose_y
        desvio_padrao_y = diferenca_y.std()
        erro_medio_y = diferenca_y.abs().mean()

        interpolated_magni_pose_yaw = interp1d(magni_pose['Time'], magni_pose['yaw'])(iris_pose['Time'])
        diferenca_yaw = iris_pose['yaw'] - interpolated_magni_pose_yaw
        desvio_padrao_yaw = diferenca_yaw.std()
        erro_medio_yaw = diferenca_yaw.abs().mean()


        erro_medio_Z = round(erro_medio_Z, 2)
        desvio_padrao_Z = round(desvio_padrao_Z, 2)
        erro_medio_x = round(erro_medio_x, 2)
        desvio_padrao_x = round(desvio_padrao_x, 2)
        erro_medio_y = round(erro_medio_y, 2)
        desvio_padrao_y = round(desvio_padrao_y, 2)
        erro_medio_yaw = round(erro_medio_yaw, 2)
        desvio_padrao_yaw = round(desvio_padrao_yaw, 2)

        teste = mv

        if teste == "sqr":
            teste = "sqr "

        # print(teste, controller, "\t",
        #       erro_medio_Z, "\t",
        #       desvio_padrao_Z, "\t",
        #       erro_medio_x, "\t",
        #       desvio_padrao_x, "\t",
        #       erro_medio_y, "\t",
        #       desvio_padrao_y, "\t",
        #       erro_medio_yaw, "\t",
        #       desvio_padrao_yaw, "\t",
        #       )
        

        print(mv, " - ", controller, "\n\t",
              "erro_medio_Z\t\t",   erro_medio_Z, "\n\t",
              "desvio_padrao_Z\t",  desvio_padrao_Z, "\n\t",
              "erro_medio_x\t\t",     erro_medio_x, "\n\t",
              "desvio_padrao_x\t",  desvio_padrao_x, "\n\t",
              "erro_medio_y\t\t",     erro_medio_y, "\n\t",
              "desvio_padrao_y\t",  desvio_padrao_y, "\n\t",
              )
        