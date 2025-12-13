import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter, butter, filtfilt

def carga_plot(nome_arquivo, NUM_PONTOS):
    dados = pd.read_table(nome_arquivo)

    dados = dados.iloc[:NUM_PONTOS, :]

    col = dados.iloc[:, 0]   # dads sensor

    # Media movel
    dados["media_movel"] = col.rolling(window=5, center=True).mean()

    # Mediana
    dados["mediana"] = col.rolling(window=5, center=True).median()

    # ewma
    dados["ewma"] = col.ewm(alpha=0.2).mean()

    # savitzky–golay
    dados["savgol"] = savgol_filter(col, 31, 2)

    # Butterworth passa-baixa
    fs = 100   # frequencia da amostragem
    cutoff = 2
    b, a = butter(3, cutoff/(fs/2), btype='low')
    dados["butter"] = filtfilt(b, a, col)


    def salva_plot(y, nome):
        plt.figure(figsize=(10,4))
        plt.plot(col, label="Original", alpha=0.4)
        plt.plot(y, label=nome, linewidth=2)
        plt.legend()
        plt.xlabel("Índice")
        plt.ylabel("Valor")
        plt.title(f"Filtro: {nome} ({NUM_PONTOS} pontos)")
        plt.tight_layout()
        plt.savefig(f"teste_filtros_300/{nome}.png")
        plt.close()

    salva_plot(dados["media_movel"], "media_movel")
    salva_plot(dados["mediana"], "mediana")
    salva_plot(dados["ewma"], "ewma")
    salva_plot(dados["savgol"], "savgol")
    salva_plot(dados["butter"], "butter")

    print("Imagens geradas com sucesso!")


carga_plot("serial3.txt", NUM_PONTOS=300)
