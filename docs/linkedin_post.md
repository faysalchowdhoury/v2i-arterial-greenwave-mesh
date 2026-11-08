Green-light speed advice cut stops by a third in my simulation and saved just 1% fuel. Then I changed one number.

I built a connected-signal testbed: signals broadcast SPaT through a lossy channel, equipped vehicles turn it into an advisory speed (GLOSA) that times their arrival on green, and an arterial microsimulation tracks stops, travel time, idling and fuel/CO2. 190 runs, 10 seeds each.

With a 300 m communication range:
🔹 100% equipped: stops −33%, fuel −1%

With a 600 m range:
🔹 100% equipped: stops −61%, fuel/CO2 −7%, travel time −1%

Why the gap? Avoiding a stop by braking early costs almost as much energy as stopping. Over 300 m a car can only shed speed by braking; over 600 m it can coast. The lead distance mattered more than going from 25% to 100% participation.

Three more findings:
1️⃣ Unequipped drivers benefit too. At 50% equipped, they stopped 45% less, because they were following advised cars through on green.
2️⃣ 30% packet loss changed nothing. Messages repeat, and one fresh one per second is enough.
3️⃣ My "smarter" queue-aware version helped at 600 m and hurt at 300 m. Waiting for the queue needs distance to act.

The takeaway for deployments: report fuel and emissions, not just stops. At short range, stops overstated the environmental benefit about 30-fold.

Caveats: fixed-time signals, single lane, perfect compliance, representative (not MOVES) fuel coefficients.

Repo: [link]

Anyone measured GLOSA fuel effects in the field? I'd like to compare.

#ConnectedVehicles #V2X #SPaT #TrafficSignals #EcoDriving #TransportationEngineering #ITS #Python
