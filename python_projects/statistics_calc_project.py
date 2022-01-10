"""
Ohjelmointi 1 tilastointia -projekti, ohjelma pyytää käyttäjältä aineistoa, kunnes tyhjä rivi syötetään tämän
jälkeen ohjelma laskee annetun aineiston avulla keskiarvon, keskihajonnan ja varianssin. Nämä arvot sitten
tulostetaan käyttäjölle tehtävänannon mukaisessa muodossa.

Atte Nyyssönen, Hervanta
"""

#tuodaan neliöjuuri laskuja varten
from math import sqrt


def keskiarvon_laskeminen(lst_data):
    """
    funktio laskee keskiarvon ja palauttaa sen arvon.
    :param lst_data: käyttäjän syöttämä aineisto.
    :return: laskettu keskiarvo.
    """
    #summataan listan arvot yhteen ja jaetaan se listan mitalla, että saadaan keskiarvo
    summa = sum(lst_data)
    keskiarvo = (summa / len(lst_data))
    return keskiarvo


def varianssin_laskeminen(lst_data):
    """
    funktio laskee varianssin ja palauttaa lasketun arvon.
    :param lst_data: käyttäjän syöttämä aineisto.
    :return: laskettu varianssin arvo.
    """
    #tuodaan keskiarvo aiemman funktion avulla
    ka = keskiarvon_laskeminen(lst_data)
    #lasketaan varianssi aineiston arvojen avulla
    varianssi = sum([(x - ka)**2 for x in lst_data]) / (len(lst_data) - 1)

    return varianssi


def keskihajonnan_laskeminen(lst_data):
    """
    funktio laskee keskihajonnan ja palauttaa sen arvon.
    :param lst_data: käyttäjän syöttämä aineisto.
    :return: laskettu keskihajonnan arvo.
    """
    #varianssi saadaan varianssin_laskeminen funktion avulla
    varianssi = varianssin_laskeminen(lst_data)
    #lasketaan keskihajonta
    keskihajonta = sqrt(varianssi)

    return keskihajonta


def tolppadiagrammin_tulostus(lst_data):
    """
    funktio tulostaa tolppadiagrammin, jossa kuvataan mittaustulosten hajautumista keskiarvon ympärillä
    tolppadiagrammi koostuu kuudesta kategoriasta.
    :param lst_data: käyttäjän syöttämä aineisto
    """
    #tarvittava keskihajonta ja keskiarvo saadaan aiempin funktioiden avulla
    kh = keskihajonnan_laskeminen(lst_data)
    ka = keskiarvon_laskeminen(lst_data)

    #muodostetaan kategoriat ja lasketaan ala- ja yläraja
    for kategorian_numero in range(0, 6):
        alaraja = kategorian_numero * 0.5 * kh
        ylaraja = (kategorian_numero + 1) * 0.5 * kh

        #tehdään lista johon tallennetaan miten monta mittaustulosta tulee mihinkin kategoriaan
        #samoin määritellään haluttu merkki kuvastamaan tulosjakaumaa
        jaottelu = []
        merkki = "*"

        #selvitetään mihin kategoriaan mittaustulos kuuluu itsearvon avulla
        for x in lst_data:
            itsearvo = abs(x - ka)

            #jos mittaustulos on pienempi kuin yläraja pysyy kategoria samana
            if alaraja <= itsearvo < ylaraja:
                sopiva_kategoria = kategorian_numero
                # tallennetaan sopivat arvot listaan
                jaottelu.append(sopiva_kategoria)


        #lasketaan tulostettavien merkkien määrä
        merkin_maara = (merkki * len(jaottelu))

        #tulostetaan tolppadiagrammi
        print(f"Values between std. dev. {alaraja:.2f}-{ylaraja:.2f}: {merkin_maara}")


def main():
    #ohjeet käyttäjälle
    print("Enter the data, one value per line.")
    print("End by entering empty line.")
    #lista johon tallennetaan annettu aineisto
    lst_data = []

    #kysytään arvoja, kunnes käyttäjä antaa tyhjän rivin
    while True:
        data = input()
        if data == "":
            break
        else:
            #tallennetaan annetut arvot listaan
            lst_data.append((float(data)))
    #jotta laskut onnistuisivat pitää arvoja olla enemmän kuin kaksi, joten tulostetaan
    #virheteksti jos arvoja syötetään liian vähän
    if len(lst_data) < 2:
        print("Error: needs two or more values")
    #edetään jos arvoja on syötetty oikea määrä
    elif len(lst_data) >= 2:
        #tulostetaan keskiarvo ja keskihajonta kahden desimaalin tarkkuudella kutsumalla funktioita
        print(f"The mean of given data was: {keskiarvon_laskeminen(lst_data):.2f}")
        print(f"The standard deviation of given data was: {keskihajonnan_laskeminen(lst_data):.2f}")
        #jos keskihajonta on 0 ei voida tulostaa tolppadiagrammia, joten tarkistetaan se ja tulostetaan
        #viesti sen mukaan, jos keskihajonta ei ole nolla kutsutaan funktiota
        if keskihajonnan_laskeminen(lst_data) == 0:
            print("Deviation is zero.")
        else:
            tolppadiagrammin_tulostus(lst_data)


if __name__ == '__main__':
    main()
