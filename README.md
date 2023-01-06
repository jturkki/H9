# H9
Arduino Nano33IoT-alustalle tehty harjoitustyo

Harjoitustyössä on jatkettu annettua ohjelmaa.
Ohjelma mittaa lämpötilaa, ilmankosteutta ja ilmanpainetta käyttäen Nano33IoT-alustan BME280 sensoria
ja valoisuutta käyttäen TSL2591 sensoria viiden minuutin välein.

Ohjelma lukee Mikkelin ulkolämpötilan openweathermap.org -palvelusta.

Ohjelma tunnistaa jos Nano33IoT-alustaa liikutetaan ja lähettää viestin siitä että sitä on liikutettu.

Kaikki yllämainitut lähetetään asksensors.com sivuston kahdelle sensorille. Alustan liikuttaminen 
sytyttää alustalle punaisen valon. Alustan nappia painamalla valo sammuu ja asksensors-sivustolle 
menee kuittaus asiasta.

Jotta ohjelma toimisi, tulee lisätä ssid-muuttujaan wifi-verkon nimi ja pass-muuttujaan verkon salasana.
Lisäksi asksensors-sivuston api-keyt pitää lisätä omiin muuttujiinsa.
