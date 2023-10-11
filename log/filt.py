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



mvs = ['line', 'sqr']
controllers = ['pd', 'cascade', 'paralel']


for mv in mvs:
    for controller in controllers:
        iris = "log/csv/" + mv + "/" + controller + "/iris_pose.csv"
        magni = "log/csv/" + mv + "/" + controller + "/magni_pose.csv"

        iris_pose = ler_csv(iris)
        magni_pose = ler_csv(magni)

        

