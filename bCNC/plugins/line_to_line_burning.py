#!/usr/bin/python
# -*- coding: ascii -*-

from __future__ import print_function
from __future__ import print_function
__author__ = ""
#__email__  = ""

__name__ = _("Line to Line Burning")
__version__ = "0.0.1"

import math
import os.path
import re
from CNC import CNC,Block
from CNCList import CNCListbox
from ToolsPage import Plugin
from math import pi, sqrt, sin, cos, asin, acos, atan2, hypot, degrees, radians, copysign, fmod
from bpath import EPS,eq,Path, Segment
from bmath import Vector
from copy import deepcopy
from operator import itemgetter
import numpy as np


class Tool(Plugin):
	__doc__ = _("""Intersection of pattern and paths""")			#<<< This comment will be show as tooltip for the ribbon button
	def __init__(self, master):
		Plugin.__init__(self, master,"Line to Line Burning")
		self.icon = "linetoline"			#<<< This is the name of file used as icon for the ribbon button. It will be search in the "icons" subfolder
		self.group = "CAM"	#<<< This is the name of group that plugin belongs
		#self.oneshot = True
		#Here we are creating the widgets presented to the user inside the plugin
		#Name, Type , Default value, Description
		self.variables = [
			("name",         "db" ,    "", _("Name")),
			("ToolSize",     "mm" ,    1, _("Laser tip size (mm)")),
			("Margin",     "float" ,    3.01234, _("Random positive float >0")),
			("In-Out", "In,Out" ,"In", _("In or Out")),
			("xAdd", "mm" , 1, _("Box X Add")),
			("yAdd", "mm" , 1, _("Box Y Add")),
			("File",       "file" ,     "", _("File to process")),
		]
		self.buttons.append("exe")

	# ----------------------------------------------------------------------
	# This method is executed when user presses the plugin execute button
	# ----------------------------------------------------------------------
	def execute(self, app):

		n = self["name"]
		if not n or n=="default": n="Pyrograph"

		#Import parameters
		toolSize = self.fromMm("ToolSize")

		if toolSize <=0:
			app.setStatus(_("Pyrograph abort: Tool Size must be > 0"))
			return

		filename = self["File"]

		#Open gcode file
		if not(filename==""):
			app.load(filename)


		inOut= self["In-Out"]
		xadd= self["xAdd"]
		yadd= self["yAdd"]
		if xadd=="":xadd=1
		if yadd=="":yadd=1

		#Create the external box
		if inOut=="Out":
			box=Block("Box")
			external_box=[]
			box.append(CNC.grapid(CNC.vars["xmin"]-xadd,CNC.vars["ymin"]-yadd))
			box.append(CNC.gline(CNC.vars["xmin"]-xadd,CNC.vars["ymax"]+yadd))
			box.append(CNC.gline(CNC.vars["xmax"]+xadd,CNC.vars["ymax"]+yadd))
			box.append(CNC.gline(CNC.vars["xmax"]+xadd,CNC.vars["ymin"]-yadd))
			box.append(CNC.gline(CNC.vars["xmin"]-xadd,CNC.vars["ymin"]-yadd))

			#Insert the external block
			external_box.append(box)
			app.gcode.insBlocks(1, external_box, "External_Box")
			app.refresh()

		#Value for creating an offset from the margins of the gcode
		margin=self.fromMm("Margin") #GIVING RANDOM DECIMALS SHOULD AVOID COINCIDENT SEGMENTS BETWEEN ISLAND AND BASE PATHS THAT CONFUSE THE ALGORITHM. WORKS IN MOST CASES.

		#Add and subtract the margin
		max_x=CNC.vars["xmax"]+margin
		min_x=CNC.vars["xmin"]-margin
		max_y=CNC.vars["ymax"]+margin
		min_y=CNC.vars["ymin"]-margin

		#Difference between offtset margins
		dx=max_x-min_x
		dy=max_y-min_y

		#Number of vertical divisions according to the toolsize
		divisions = dy / toolSize 

		#Distance between horizontal lines
		step_y = toolSize
		n_steps_y =int(divisions)+1


		#Create the snake pattern according to the number of divisions
		pattern=Block(self.name)
		pattern_base=[]

		for n in range(n_steps_y):
			if n==0:
				pattern.append(CNC.grapid(min_x,min_y)) #go to bottom left
			else:
				y0=min_y+step_y*(n-1)
				y1=min_y+step_y*(n)
				if not(n%2 == 0):
					pattern.append(CNC.glinev(1,[max_x,y0])) #write odd lines from left to right
					pattern.append(CNC.grapid(max_x,y1))
				else:
					pattern.append(CNC.glinev(1,[min_x,y0])) #write even lines from right to left
					pattern.append(CNC.grapid(min_x,y1))

		#Insert the pattern block
		pattern_base.append(pattern)
		app.gcode.insBlocks(1, pattern_base, "pattern")
		app.refresh()
		
		#Mark pattern as island
		for bid in pattern_base:
			app.gcode.island([1])

		#Select all blocks
		app.editor.selectAll()

		paths_base = []
		paths_isl = []

		points=[]

		#Compare blocks to separate island from other blocks
		for bid in app.editor.getSelectedBlocks():
			if app.gcode[bid].operationTest('island'):
				paths_isl.extend(app.gcode.toPath(bid))
			else:
				paths_base.extend(app.gcode.toPath(bid))

		#Make intersection between blocks
		while len(paths_base) > 0:
			base = paths_base.pop()
			for island in paths_isl:
				#points.extend(base.intersectPath(island))
				points.extend(island.intersectPath(base))


###SORTING POINTS###
		x=[]
		y=[]

		#Get (x,y) intersection points  
		for i in range(len(points)):
			x.append(points[i][2][0])
			y.append(points[i][2][1])

		#Save (x,y) intersection points in a matrix
		matrix=[[0 for i in range(2)] for j in range(len(x))]

		for i in range(len(x)):
			matrix[i][0]=x[i]
			matrix[i][1]=y[i]

		# for i in range(len(x)):
		# 	print('puntos',points[i][0],points[i][1],matrix[i][0], matrix[i][1])

		#print(matrix)

		#Sort points in increasing y coordinate
		matrix.sort(key=itemgetter(1,0), reverse=False)

		# for i in range(len(x)):
		# 	print('puntos',points[i][0],points[i][1],matrix[i][0], matrix[i][1])
		#print(matrix)

		index=0
		pair=0
		new_matrix=[]
		for i in range(len(x)):	
			if i==0:
				index=index+1
			elif i<len(x)-1:
				if matrix[i][1]==matrix[i-1][1]:
					index=index+1
				else:
					if pair%2==0:
						logic=False
					else:
						logic=True
					submatrix=matrix[i-index:i]
					submatrix.sort(key=itemgetter(0,1), reverse=logic)
					new_matrix.extend(submatrix)
					index=1
					pair=pair+1
			else:
				#print('entered')
				if pair%2==0:
					logic=False
				else:
					logic=True
				if matrix[i][1]==matrix[i-1][1]:
					submatrix=matrix[i-index:]
					submatrix.sort(key=itemgetter(0,1), reverse=logic)
					new_matrix.extend(submatrix)
				else:
					index=1
					submatrix=matrix[-1]
					new_matrix.extend(submatrix)

		# for i in range(len(x)):
		# 	print('puntos',new_matrix[i][0], new_matrix[i][1])

		# for i in range(len(x)):
		# 	print('puntos',new_matrix[i][0], new_matrix[i][1])
		#print(x, y)

###SORTING POINTS END###

		#Generate the gcode from points obtained
		blocks = []
		block = Block(self.name)
	
		for i in range(len(x)):
			if i == 0:
				block.append(CNC.grapid(new_matrix[0][0],new_matrix[0][1]))
				inside=0
			else:
				if inside==0:
					block.append(CNC.gline(new_matrix[i][0],new_matrix[i][1]))
					inside=1
				else:
					block.append(CNC.grapid(new_matrix[i][0],new_matrix[i][1]))
					inside=0	


		# for i in range(len(x)):
		# 	print('puntos',x[i], y[i])

		blocks.append(block)
		app.gcode.delBlockUndo(1)
		app.gcode.insBlocks(1, blocks, "Line to Line Burning")
		
		app.editor.disable()

		for block in blocks:
			block.enable=1

		#app.editor.enable()
		#app.editor.unselectAll()
		app.refresh()
		app.setStatus(_("Generated Line to Line Burning"))

##############################################

