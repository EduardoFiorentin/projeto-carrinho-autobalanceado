import serial
import time
import sys

# Configurações da porta serial
SERIAL_PORT = '/dev/ttyUSB0'  # Altere para a porta correta (ex: '/dev/ttyUSB0' no Linux, 'COM3' no Windows)
BAUD_RATE = 115200      # Altere para a taxa de transmissão (baud rate) do seu dispositivo
OUTPUT_FILE = 'serial3.txt' # Nome do arquivo de saída

def read_serial_and_save():
    try:
        # Abre a porta serial
        # O 'with' garante que a porta será fechada automaticamente, mesmo em caso de erro
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Conectado a: {ser.portstr}")
            # Abre o arquivo para escrita em modo 'append' ('a'), para adicionar novas linhas
            # Usar 'w' sobrescreveria o arquivo a cada execução
            with open(OUTPUT_FILE, 'a', encoding='utf-8') as f:
                print(f"Salvando dados em: {OUTPUT_FILE}. Pressione Ctrl+C para parar.")
                
                while True:
                    # Lê uma linha da porta serial
                    line_bytes = ser.readline()
                    
                    if line_bytes:
                        try:
                            # Decodifica os bytes para string, removendo caracteres de nova linha (\r\n)
                            line = line_bytes.decode('utf-8').strip()
                            if line:
                                print(f"Recebido: {line}")
                                # Escreve a linha no arquivo, seguida de uma quebra de linha
                                f.write(line + '\n')
                                # Garante que os dados sejam imediatamente gravados no disco
                                f.flush() 
                        except UnicodeDecodeError:
                            print("Erro ao decodificar a linha (possíveis caracteres não UTF-8)")

    except serial.SerialException as e:
        print(f"Erro de porta serial: {e}")
        print(f"Verifique se a porta '{SERIAL_PORT}' está correta e disponível.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nLeitura interrompida pelo usuário.")
    except Exception as e:
        print(f"Ocorreu um erro inesperado: {e}")
    finally:
        print("Programa encerrado.")

if __name__ == "__main__":
    read_serial_and_save()