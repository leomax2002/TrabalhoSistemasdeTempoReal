import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import colors 
from matplotlib.ticker import PercentFormatter


n_bins = 40

  
files = ["tempoDeRespostaShowScreen.txt","tempoDeRespostareadSensor.txt","tempoDeRespostaOpenValve.txt","tempoDeRespostaCloseValve.txt","tempoDeRespostaButtonInterrupt.txt"]
arr = []
for f in files:
    with open(f) as file:
        for line in file:
            try:
                arr.append(int(line.strip()))
            except:
                continue

    fig, axs = plt.subplots(1, 1,tight_layout = True)
    N, bins, patches = axs.hist(arr, bins=n_bins)
    fracs = N/N.max()
    norm = colors.Normalize(fracs.min(), fracs.max())
    for thisfrac, thispatch in zip(fracs, patches):
        color = plt.cm.viridis(norm(thisfrac))
        thispatch.set_facecolor(color)
    axs.hist(arr, bins = n_bins,density = True)
    aux_title = f.split("tempoDeResposta")[1].split(".txt")[0]
    #print(aux_title)
    axs.set_title("Tempo de Resposta Tarefa " + aux_title)
    axs.set_ylabel("Frequência")
    axs.set_xlabel("Tempo de Execução em Microssegundos")
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
