import { useState } from "react"
import './App.css' 

function App() { 
  const apiKey = "e45349b2efd50bbe4652b6fb8ecdb057" //specific key for each persons account
  const [city, setCity] = useState("") //creating a state variable to store what user types in
  const [weather, setWeather] = useState(null)// creating a state variable to store data from the api

  const handleSearch = async () => { //async means the function will always return a promise
    if (!city) return //if user doesnt enter a city the function stops
    console.log("Searching for city:", city)


    try { //await means to pause here until the promise is done then continue
      const response = await fetch( //I promise i will ... but not yet, then when promise is done stored in res
        `https://api.openweathermap.org/data/2.5/weather?q=${city}&appid=${apiKey}&units=metric`
      )
      const data = await response.json() //converts api response to json
      

      if (data.cod === 200) { //200 means everything worked fine, cod is short for code from my api
        // if the city is found
        setWeather({
          temp: data.main.temp, 
          wind: data.wind.speed,
          rain: data.rain ? data.rain["1h"] : 0, //if its raning uses after the ? if not prints 0
          name: data.name,
        })
      } else {
        // City not found or other API error
        setWeather({ error: `${city} not found` }) //sets zn error proopertty
      }
    } catch (err) {
      console.error(err)
      setWeather({ error: "An error occurred. Please try again." }) 
    }
  }

  const cropinfo = (weather) => { //declare function based on the weather
    const tempstatus =
      weather.temp >= 15 && weather.temp <= 30
        ? "✅ Temperature ideal for crops" //shorter way of writing if else and it works becausei only have 2 cases
        : "⚠️ Temperature may stress crops"

    const rainstatus =
      weather.rain <= 2
        ? "✅ Rainfall sufficient, if rain continues watch for flooding"
        : "⚠️ Rainfall low, consider irrigation"

    const windstatus =
      weather.wind <= 10
        ? "✅ Wind safe for crops"
        : "⚠️ Windy, crop damage may occur"

    return ( // ui interface stuff
      <div
        style={{
          border: "1px solid",
          padding: "1rem",
          marginTop: "1rem",
          borderRadius: "8px",
          maxWidth: "400px",
        }}
      >
        <h3>Crop Information 📊</h3>
        <p>{tempstatus}</p>
        <p>{rainstatus}</p>
        <p>{windstatus}</p>
      </div>
    )
  }

  return (
    <div style={{ fontFamily: "sans-serif", padding: "2rem" }}>
      {/* main heading */}
      <h1>Weather App</h1> 
      <h3 style={{ fontWeight: "normal", marginTop: "0.5rem" }}>
        {/* subheading */}
        Enter any city worldwide to view its current weather and crop-related information.
      </h3>

      <input
        type="text"
        placeholder="Enter a city"
        value={city}
        onChange={(e) => setCity(e.target.value)} // e is the event, e.target.value is the current text inside, setcity updates the statre
      />
  

      {/* wwhen button is clicked run the function */}
      <button onClick={handleSearch} style={{ padding: "0.5rem 1rem" }}> 
        {/* the text shown on the button */}
        Search
      </button>

      {/* Eprints the error message if weather.error exsists*/}
      {weather && weather.error && (
        <p style={{ color: "red", marginTop: "1rem" }}>{weather.error}</p>
      )}

      {/* only continure if weather isnt null or undefined and check no error stored in wather and wont show this section*/}
      {weather && !weather.error && (
        <div style={{ marginTop: "1rem" }}>
          <h2>Current weather in: {weather.name}</h2>
          <p>🌡 Temp: {weather.temp}°C</p>
          <p>💨 Wind: {weather.wind} m/s</p>
          <p>🌧 Rain: {weather.rain} mm</p>

          {cropinfo(weather)}
        </div>
      )}
    </div>
  )
}

export default App

// 20% self written
// References 
// https://openweathermap.org/current
// https://www.youtube.com/watch?v=iM4Pz-9oyiE
// https://www.youtube.com/watch?v=je3FTTunyp4&t=1304s
// https://www.youtube.com/watch?v=ib8MHSMwtYg&t=168s
// https://www.youtube.com/watch?v=atS_A9HHAVo
// https://chatgpt.com

