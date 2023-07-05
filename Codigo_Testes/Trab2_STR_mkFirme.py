import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import colors 
from matplotlib.ticker import PercentFormatter


n_bins = 40

  
files = ["tempoDeRespostaShowScreen2.txt","tempoDeRespostareadSensor2.txt","tempoDeRespostaOpenValve2.txt","tempoDeRespostaCloseValve2.txt","tempoDeRespostaButtonInterrupt2.txt"]
arr = []
for f in files:
    with open(f) as file:
        for line in file:
            try:
                arr.append(int(line.strip()))
            except:
                continue

    #fig = plt.figure()
    print(arr)
    aux_title = f.split("tempoDeResposta")[1].split(".txt")[0].split("2")[0]
    plt.title("Tempo de Resposta Tarefa " + aux_title)
    plt.xlabel("Casos de Teste")
    plt.ylabel("Tempo de Execução em Microssegundos")
    
    plt.bar([x for x in range(len(arr))],arr)
    plt.show() 
    hwm = np.max(arr)
    mean = np.mean(arr)
    minimal = np.min(arr)
    if f == "tempoDeRespostaShowScreen.txt" or f == "tempoDeRespostaButtonInterrupt.txt":
        wcrt = hwm*1.2
    else:
        wcrt = hwm*1.5
    print(f)
    print("HWM ",hwm)
    print("mean ",mean)
    print("min ",minimal)
    print("wcrt ",wcrt)
    arr.clear()