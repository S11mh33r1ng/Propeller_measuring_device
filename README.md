# Propeller_measuring_device (Vajab uuendamist!)

Lühike stendi kasutamise õpetus

1. Enne USB ühendamist lülita pink sisse
2. Ühenda USB
3. Käivita kasutajaliides töölaua ikooni abil
4. Ühenduse loomiseks vali korretkne port (vajadusel) ja vajuta "Ühenda"
5. Vajuta "Säti parameetrid", kontrolli parameetrid (eelkõige koormusandurite info) ning kinnita nii algparameetrid kui telgede parameetrid
6. Otsi välja kasutatava propelleri konfiguratsiooni fail
7. Sisesta korrektne propelleri läbimõõt
8. Sisesta korrektne mõõtmiskaugus (suhtarv propelleri raadiusesse) - see on vajalik õhuvoo joa monitoorimiseks. 
9. Kui mõõtmiskauguseks on 0R ehk mõõdetakse propelleri tagaserva lähedalt õhuvoogu, siis tuleb kasutada trajektoori faili, et vältida kokkupõrget propelleriga. Trajektoori faili valimiseks avada "Trajektoori valikute" aken ja "Otsi olemasolev trajektoori fail". Trajektoorid asuvad töölaual trajektooride kaustas. Pärast valikut võib akna sulgeda.
10. Lülitada sisse toiteplokk ja sättida parameetrid. Helikopteri ESCi kasutades lülitada sisse ESCi jahutusventilaator.
11. Kui ei ole eelnevalt teada propelleri kiiruse protsenti, siis teha test. Valida kiiruse väärtus ja vajutada "Testi mootorit". Kiiruse väärtust saab töö ajal muuta. Kontrollida testi ajal ka koormusandurite väärtuseid. Vajadusel teha tarkvarale ja pingile taaskäivitus ja alustada kogu tsükliga uuesti.
12. Kui kiiruse väärtused on teada, siis vajutada "Pitot' tsentrisse"
13. Mõõta välja korrektne vahemaa propelleri rummu ja Pitot vahel. Kui on trajektoori faili abil liikumine, siis on soovitatav jätta ca 5-8 mm tagaserva ja Pitot vahele ruumi. Teistel juhtudel siis vastavalt propelleri raadiuse ja kauguse suhtele. Propelleri kaugust saab muuta Y-telje liigutamise ning propelleri raami liigutamise teel.
14. Määrata ära kordusmõõtmiste arv (tavaliselt on 10)
15. Vajutada "Alusta mõõtmisega"
16. Kui mõõtmised on tehtud, siis avada logi kaustast viimase kellaajaga kaust, avada _mean.csv fail OpenOffice Calc'is ja salvestada fail .xlsx laiendiga. 
17. Laadida tulemused üles Google Drive'i aerodünaamika uurimisgrupi mõõtetulemuste kausta. Kausta nimes võiks minimaalselt olla kirjas propi diameeter, rummu nurk, RPM ja mõõtekaugus (0R-2R)
18. Sulgeda või taaskäivitada kasutajaliides
