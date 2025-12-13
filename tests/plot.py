import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, butter, filtfilt
import numpy as np

def carga_plot(nome_arquivo):
   
    dados = pd.read_table(nome_arquivo)
    eixo_x = dados.columns[:]

    dados['media_2_valores'] = (dados.iloc[:, 0].shift(1) + dados.iloc[:, 0].shift(-1)) / 2
    dados['media_4_valores'] = (dados.iloc[:, 0].shift(2) + dados.iloc[:, 0].shift(1) + dados.iloc[:, 0].shift(-1) + dados.iloc[:, 0].shift(-2)) / 4
    dados['media_6_valores'] = ( dados.iloc[:, 0].shift(3) + dados.iloc[:, 0].shift(2) + dados.iloc[:, 0].shift(1) + dados.iloc[:, 0].shift(-1) + dados.iloc[:, 0].shift(-2) + dados.iloc[:, 0].shift(-3)) / 6
    dados['media_7_valores'] = ( dados.iloc[:, 0].shift(3) + dados.iloc[:, 0].shift(2) + dados.iloc[:, 0].shift(1) + dados.iloc[:, 0].shift(0) + dados.iloc[:, 0].shift(-1) + dados.iloc[:, 0].shift(-2) + dados.iloc[:, 0].shift(-3)) / 7


    # tipos: ('line', 'bar', 'barh', 'kde', 'density', 'area', 'hist', 'box', 'pie', 'scatter', 'hexbin')
 
    fig, ax = plt.subplots()

    # dados.iloc[:300, 0].plot(kind='line', label="Sensor", color="red")
    # dados.iloc[:5000, 1].plot(kind='line', label="res. PID", color="red")
    
    # Média 1 valor cada lado (2 valores)
    # dados.iloc[:300, 4].plot(kind='line', label="res. PID", color="red")
    
    # Média 2 valores cada lado (4 valores)
    # dados.iloc[:300, 5].plot(kind='line', label="ajuste", color="red")
    
    # Média 3 valores cada lado (6 valores)
    # dados.iloc[:300, 6].plot(kind='line', label="ajuste", color="red")

    # Média 3 valores cada lado mais central(7 valores)
    dados.iloc[:300, 6].plot(kind='line', label="ajuste", color="red")
    # dados.iloc[:1000, 3].plot(kind='line', label="res. PWM", color="black")

    # Adiciona títulos e exibe o gráfico
    plt.title('média 3 valor cada lado mais central (7 valores)')
    plt.xlabel('Índice')
    plt.ylabel('Valores')
    ax.legend(loc='upper left')
    plt.ylim(top=1) 
    plt.ylim(bottom=-1)
    # plt.show()
    plt.savefig("ruido_3_val_cada_lado_e_cental.png")


# Chama a função com o nome do seu arquivo
carga_plot('serial3.txt')