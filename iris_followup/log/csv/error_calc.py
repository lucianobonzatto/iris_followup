import numpy as np
import pandas as pd
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw
from scipy.interpolate import interp1d

def ler_csv(nome_arquivo):
    try:
        # Lendo o arquivo CSV
        df = pd.read_csv(nome_arquivo)
        return df
    except FileNotFoundError:
        print("Arquivo não encontrado.")
        return None

def plot_grafico3d(ax, controller, vel):
    iris_pose = ler_csv("log/csv/" + controller + vel + "/iris_pose.csv")
    magni_pose = ler_csv("log/csv/" + controller + vel + "/magni_pose.csv")
    
    ax.plot(iris_pose["X Position"].to_numpy(), iris_pose["Y Position"].to_numpy(), iris_pose["Z Position"].to_numpy(), c='b', label=f'iris_pose')
    ax.plot(magni_pose["X Position"].to_numpy(), magni_pose["Y Position"].to_numpy(), magni_pose["Z Position"].to_numpy(), c='r', label=f'magni_pose')
    
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')

    ax.set_xlim(0, 4)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 2.1)
    
    ax.legend()

def distancia_entre_pontos(p1, p2):
    return euclidean(p1, p2)

velocitys = ["03", "04", "05"]
controllers = ['pd', 'cascade', 'paralel']

for controller in controllers:
    for velocity in velocitys:
        iris_pose = ler_csv("log/csv/" + controller + velocity + "/iris_pose.csv")
        magni_pose = ler_csv("log/csv/" + controller + velocity + "/magni_pose.csv")

        x = iris_pose["X Position"].to_numpy()
        y = iris_pose["Y Position"].to_numpy()
        iris_trajetoria = np.array([x, y]).T

        x = magni_pose["X Position"].to_numpy()
        y = magni_pose["Y Position"].to_numpy()
        magni_trajetoria = np.array([x, y]).T

        num_pontos = len(iris_trajetoria)
        
        interpolador = interp1d(np.linspace(0, 1, len(magni_trajetoria)), magni_trajetoria, axis=0)
        trajetoria_gostaria_interpolada = interpolador(np.linspace(0, 1, num_pontos))

        eqm = sum((distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) ** 2 for i in range(num_pontos))) / num_pontos
        eam = sum(distancia_entre_pontos(trajetoria_gostaria_interpolada[i], iris_trajetoria[i]) for i in range(num_pontos)) / num_pontos

        erro_maximo_x = max(abs(trajetoria_gostaria_interpolada[i][0] - iris_trajetoria[i][0]) for i in range(num_pontos))
        erro_maximo_y = max(abs(trajetoria_gostaria_interpolada[i][1] - iris_trajetoria[i][1]) for i in range(num_pontos))

        print(controller + " " + velocity +
              " - EQM: " + str(eqm) + 
              " - EAM: " + str(eam) + 
              " - erro_maximo_x: " + str(erro_maximo_x) + 
              " - erro_maximo_y: " + str(erro_maximo_y))
    print("")

