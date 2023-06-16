#!/usr/bin/env python3

import csv

import numpy as np
import matplotlib.pyplot as plt

# indices of data in the input file
inds = {(leg,part) : [13 + leg*4 + part, 25 + leg*4 + part]
        for leg in range(3)
        for part in range(4)}

with open('../output/measurements.csv', 'r') as f:
    rs = csv.reader(f, delimiter=',', quotechar='"')
    raw_data = [row 
            for i,row in enumerate(rs)
            if i > 0]

data_by_part = {(leg,part) : [row[pos]
                              for pos in poss
                              for row in raw_data]
    for (leg,part),poss in inds.items()}

add_2_last = False

if add_2_last:
    proc_data = {}
    for leg in [0,1,2]:
        for part in [0,1]:
            proc_data[(leg, part)] = data_by_part[(leg, part)]
        proc_data[(leg, 2)] = [v1 + '+' + v2    
            for v1,v2 in zip(data_by_part[(leg, 2)], data_by_part[(leg, 3)])]
    data_by_part = proc_data

print('data_by_part: {}'.format(data_by_part))

# filter data

filtered_data = {}

raw_results = []

for (leg, part), vals in data_by_part.items():
    correct_values = []
    im_res = []
    for v in vals:
        try:
            new_v = float(v)
            correct_values.append(new_v)
            im_res.append('{:.3f}'.format(new_v))
        except ValueError:
            # v is not a number; trying to evaluate v
            ch_v = v.replace(',', '.')
            try:
                new_v = eval(ch_v)
                correct_values.append(new_v)
                im_res.append('{:.3f}'.format(new_v))
            except (SyntaxError, TypeError):
                print(ch_v)
                im_res.append('None')
    filtered_data[(leg, part)] = correct_values
    raw_results.append(im_res)

#print('filtered_data: {}'.format(filtered_data))

parts = [0,1,2] if add_2_last else [0,1,2,3]
print(raw_results)
with open('out.csv', 'w') as f:
    f.write(';'.join(['{}_{}'.format(leg,part) for leg in [0,1,2] for part in parts]) + '\n')
    for ind in range(len(raw_results[0])):
        f.write(';'.join([rr[ind] for rr in raw_results]) + '\n')

# gather data for each leg
result_data = {}
for leg in [0, 1, 2]:
    result_data[leg] = [filtered_data[leg,p] for p in parts]

# extremums
max_v = max([max(vs) for k,vs in filtered_data.items()]) + 0.2
min_v = min([min(vs) for k,vs in filtered_data.items()]) - 0.2

fig, axs = plt.subplots(3, 1)
parts = ['тазик', 'бедро', 'голень и лапка'] if add_2_last else ['тазик', 'бедро', 'голень', 'лапка']
title = {0: 'Передняя пара ног',
1: 'Средняя пара ног',
2: 'Задняя пара ног'}
for leg in [0, 1, 2]:
    axs[leg].boxplot(result_data[leg])
    axs[leg].set_ylim((min_v, max_v))
    axs[leg].set_xticks(range(1, 1+len(result_data[leg])), parts)
    #axs[leg].set_title(title[leg])
    axs[leg].grid(True, 'both', axis='y')
    axs[leg].set_ylabel('длина, мм')

plt.show()

# create plot for velocities

with open('../output/res.csv', 'r') as f:
    rs = csv.reader(f, delimiter=';')
    data = [[float(row[1]), float(row[3])] 
            for i,row in enumerate(rs)
            if i > 0]
    
xs = list(set([d[0] for d in data]))
xs.sort()

fs = [1. / v for v in xs]

ys = []
for x in xs:
    vs = [d[1] for d in data if d[0] == x]
    ys.append(sum(vs) / len(vs) * 1000)

plt.plot(fs[:-1], ys[1:], 'o-')
plt.xlabel('частота шагания, Гц')
plt.ylabel('скорость, мм/с')
plt.show()
