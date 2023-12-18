Dokument obsahuje zdrojový kód každého vytvoreného algortimu k Diplomovej práci

Jednotlivé algoritmy sú umiestnené v samostatných priečinkoch.
Nakoľko existujú jemné rozdieli v zobrazovaní a spracovaní dát, 
sú algoritmy rozdelené na 3 časti (SEF na 2) oddelujúcou príponou :
							
	_vytvoreneData 	: Algoritmy spracovávajú zvolené vytvorené dáta napr. ("imgs/pointCloud4.png")
	
	_jednoMeranie 	: Algoritmy spracovávajú jedno z 107 experimentálnych meraní napr.[63]

	_viaceroMerani 	: Algoritmy spracovávajú množinu viacerých spojených experimentálnych meraní napr. [0:20] 

Priečok data_python obsahuje súbory s meranými dátami.
Priečok imgs obsahuje súbory s vytvorenými dátami.

Algoritmy SplitandMerge, Ransac a Hough spracovávajú ako vstupné dáta obrázok z bitmapy, ktorý si prevedú.
Je tomu tak, aby bola dokázaná ich schopnosť spracovať aj takéto vstupné dáta a schopnosť spracovávať neusporiadané pole bodov. 
V časti meraných dát, spracovávajú už vstupné dáta vo forme poľa bodov.


Diplomová práca 2021
Marek Šooš
