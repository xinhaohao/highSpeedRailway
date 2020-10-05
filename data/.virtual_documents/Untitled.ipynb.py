import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


datas = pd.read_csv("diff.csv")


plt.figure()

plt.plot(datas.iloc[:, 6])



