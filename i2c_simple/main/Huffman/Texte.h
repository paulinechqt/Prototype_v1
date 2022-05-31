#ifndef PROJET_TEXTE_H
#define PROJET_TEXTE_H
#include<cmath>
#include<string>
#include<iostream>
#include<vector>
#include<queue>
#include<fstream>
#include<map>

using namespace std;

class Texte {
    struct noeud { //Déclaration d'un type de variable de type noeud
        char s_donnee; //Caractère à coder
        double s_freq; //Fréquence du caractère
        noeud *s_gauche; //Adresse du fil gauche
        noeud *s_droit; //Adresse du fil droit
        noeud (char d, double f)
        {
            s_gauche = NULL;
            s_droit = NULL;
            this -> s_donnee = d; //Modification de s_donnee
            this -> s_freq = f; //Modification de s_freq
        }
    };

    struct compare //Compare les fréquences de 2 noeuds
    {
        bool operator() (noeud *gauche, noeud *droit)
        {
            return gauche -> s_freq > droit -> s_freq;
        }
    };

    noeud *m_rac; //racine
    map<char,string> m_codes; //Lettres associées à leur code binaire
    string m_texteBinaire; //Texte sous forme de chaîne binaire
    string m_texteASCII; //Chaîne extraite du fichier texte
    int m_profondeurMax=0; //Code binaire le plus long
    string m_texteDecode; //Chaîne décodée

public:
    Texte() //Constructeur
    {
        m_rac = NULL;
    }

    ~Texte() //Destructeur
    {
        delete m_rac;
        m_rac=0;
        m_codes.clear();
    }

    void Encodage()
    {
        for (int i=0; i <= m_texteASCII.size(); i++) //Pour tous les caractères de la table ASCII
        {
            m_texteBinaire += m_codes[(m_texteASCII[i])]; //Remplacement des lettres par leur code binaire
        }
    }

    void Enregister(string path) //Enregistrement dans un fichier texte
    {
        ofstream FichierDecode(path.c_str());
        FichierDecode << Decodage() << endl;
    }

    string LectureFichier(string path) //Lecture du fichier texte et récupération de la chaîne de caractères
    {
        ifstream monFichier(path.c_str());
        getline(monFichier, m_texteASCII); //lecture du fichier ligne par ligne et sauvegarde
        Frequences();
        return m_texteASCII;
    }

    int PlusGrandeProfondeur(string binaire) //Calcul du plus grand code binaire
    {
        if (binaire.size() > m_profondeurMax)
        {
            m_profondeurMax = binaire.size();
           return m_profondeurMax;
        }
        else
        {
            return m_profondeurMax;
        }
    }

    string Decodage()
    {
        int compteur = 0;
        while (compteur <= (m_texteBinaire.size()-m_profondeurMax ))
        {
            for (int j=1; j <= m_profondeurMax; j++)
            {
                for (auto pair : m_codes)
                {
                    if (pair.second == m_texteBinaire.substr(compteur,j)) //Si un code correspond à une valeur dans la map
                    {
                        m_texteDecode += pair.first; //alors on concatène la lettre avec le début du texte décodé
                        compteur += j;
                    }
                }
            }
        }
        cout << m_texteASCII << " -> " << m_texteDecode << endl;
        return m_texteDecode;
    }

    void CodesBinaires (struct noeud *racine, string binaire) //Stocke les mots binaires
    {
        if (!racine) //Racine existante
        {
            return;
        }
        if (racine -> s_donnee != NULL) //Racine non existante
        {
            m_codes.insert(pair<char,string> (racine -> s_donnee, binaire)); //ajout à la map du code binaire
            PlusGrandeProfondeur(binaire); //calcul pour savoir si ce nouveau code binaire est plus long
        }
        CodesBinaires(racine->s_gauche, binaire + "0");
        CodesBinaires(racine->s_droit, binaire + "1");
    }

    void Afficher() //Affiche les mots binaires
    {
        for (auto pair: m_codes)
        {
            cout << pair.first << " " << pair.second << '\n';
        }
    }


    void Frequences() //Compte la fréquence de chaque caractère dans une chaîne
    {
        vector <char> caractere (0); //vector pour tous les caractères
        vector <double> frequence (0); //Vector pour toutes les fréquences

        for (int i = 0; i <= 127; i++) //Pourcours toute la table ASCII
        {
            char ascii = static_cast<char>(i); //code hexa à caractère ASCII

            if (count(m_texteASCII.begin(), m_texteASCII.end(), ascii) >= 1) //Si ième caractère de la table ASCCI se trouve dans le fichier texte à encoder
            {
                caractere.push_back(ascii); //alors on ajoute la caractère au vector caractere
                frequence.push_back(count(m_texteASCII.begin(), m_texteASCII.end(), ascii)); //et on ajoute sa fréquence au vector frequence
            }
        }
        int taille  = caractere.size();
        HuffmanCodes(caractere, frequence, taille); //Création de l'arbre
    }

    //Créé l'arbre
    void HuffmanCodes (vector<char> d, vector<double> f, int taille) //taille: nb de caractères différents
    {
        noeud *droit, *gauche, *top; //Création des noeuds droit, gauche et somme

        priority_queue <noeud*, vector <noeud*>, compare> queue; //Création de la file de priorité

        for (int i = 0; i< taille; i++)
        {
            queue.push(new noeud(d[i], f[i])); //Ajout de toutes les valeurs et fréquences dans un vecteur
        }

        while (queue.size() != 1) //Tant que la file de priorité possède plus de 1 élément
        {
            gauche = queue.top(); //Plus petite fréquence de la file sur le fil gauche
            queue.pop(); //Suppression de l'élément le plus prioritaire de la file

            droit = queue.top(); //2e plus petite fréquence de la file sur le fil droit
            queue.pop(); //Suppression du 2e élément le plus prioritaire de la file

            top = new noeud (NULL, gauche -> s_freq + droit -> s_freq); //Création d'un nouveau noeud (somme des 2 précédents)

            top -> s_gauche = gauche;

            top -> s_droit = droit;

            queue.push(top); //Ajout du noeud de somme dans la file de priorité
        }
        CodesBinaires(queue.top(), "");
        Afficher();
        Encodage();
    }
};

#endif //PROJET_TEXTE_H