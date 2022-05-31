#include "Texte.h"
#include<string>

using namespace std;

int main() {

    //Chemin vers le fichier d'écriture
    // /!\ Ne pas mettre de point dans le texte (seulement à la fin)
    // /!\ Terminer le texte par un point

    string const pathFichier_A_Decode ("C:\\Users\\pauli\\CLionProjects\\Programmation\\Projet version 4\\Fichier_Texte");

    //Chemin vers le fichier de décodage
    string const pathFichier_Decode("C:\\Users\\pauli\\CLionProjects\\Programmation\\Projet version 4\\Fichier_Texte_Decode");

    Texte t;
    t.LectureFichier(pathFichier_A_Decode); //Lecture du fichier, création de l'arbre et codage du texte
    t.Enregister(pathFichier_Decode); //Décodage et enregistrement

    return 0;


}
