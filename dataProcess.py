#!/usr/bin/env python3
import csv

def initCSV(where):
    headers = ['gen', 'key']
    f = open(where, 'w', encoding='utf-8')
    f_csv = csv.writer(f)
    f_csv.writerow(headers)
    f.close()

def csvGenerator(gen, key, where):
    row = [gen, key]
    f = open(where, 'a', encoding='utf-8')
    f_csv = csv.writer(f)
    f_csv.writerow(row)
    f.close()
