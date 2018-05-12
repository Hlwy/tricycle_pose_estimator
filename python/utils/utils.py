# Created by: Hunter Young
# Date: 4/24/18
#
# Script Description:
# 	TODO
# Current Recommended Usage: (in terminal)
# 	TODO

import numpy as np
import os
import csv


def export_list2csv(_path, _file, _headers, _datalist):
	filenames = os.path.split(_file)
	filename = filenames[-1]
	print(filename)

	if not os.path.exists(str(_path)):
		print("Target output directory [" + str(_path) + "] does not exist --> MAKING IT NOW")
		os.makedirs(_path)

	csvFile = str(_path) + "/" + str(filename) + ".csv"
	with open(csvFile, "w") as output:
		writer = csv.writer(output, lineterminator='\n')
		writer.writerow(_headers)

		for row in range(len(_datalist)):
			tmpData = _datalist[row]
			writer.writerow(tmpData)

	print("	Data exporting to ...")

def import_csv2list(_filepath):
	data = []
	with open(_filepath, 'rb') as sd:
		r = csv.DictReader(sd)
		for line in r:
			data.append(line)
	return data

def add_filename_prefixs(_dir, _prefix):
	filenames = os.listdir(_dir)
	os.chdir(_dir)
	for file in filenames:
		newName = str(_prefix) + "_" + str(file)
		os.rename(file, newName)
		# print(file)
		# print(newName)
	# print(filenames)
	print("Finished")
