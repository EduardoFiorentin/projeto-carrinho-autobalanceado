import pandas as pd
import matplotlib.pyplot as plt

def carga_plot(nome_arquivo):
   
    dados = pd.read_table(nome_arquivo)
    eixo_x = dados.columns[:]

    # tipos: ('line', 'bar', 'barh', 'kde', 'density', 'area', 'hist', 'box', 'pie', 'scatter', 'hexbin')
 
    fig, ax = plt.subplots()

    dados.iloc[:5000, 0].plot(kind='line', label="Sensor", color="blue")
    dados.iloc[:5000, 1].plot(kind='line', label="res. PID", color="red")
    # dados.iloc[:1000, 3].plot(kind='line', label="res. PWM", color="black")

    # Adiciona títulos e exibe o gráfico
    plt.title('')
    plt.xlabel('Índice')
    plt.ylabel('Valores')
    ax.legend(loc='upper left')
    # plt.ylim(top=1) 
    # plt.ylim(bottom=-1)
    # plt.show()
    plt.savefig("serial3_line_5000.png")


# Chama a função com o nome do seu arquivo
carga_plot('serial3.txt')