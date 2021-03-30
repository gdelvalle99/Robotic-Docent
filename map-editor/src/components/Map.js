import React, { useState, useEffect } from "react";
import { mapLink } from '../links';
import axios from 'axios';
import { PieceButton } from "./PieceButton";

export const Map = (props) => {

    const [image, setImage] = useState("");
 
    const getImage = () => {
        const formData = new FormData();
        formData.append('floor_id', '476411ee-df58-4b98-8da6-4514d2fc0433'); // just for testing

        const token = localStorage.getItem('auth_token') || "";
        fetch(mapLink
            , {
                method: 'POST',
                headers: {
                    'Accept': 'application/json, text/plain, */*',
                    "Authentification": token,
                    "credentials": true
                },
                body: formData
            }
        ).then(response => {
            return response.blob()
        }).then(blob => {
            const link = URL.createObjectURL(blob)
            setImage(link)
        }).catch(e=>console.log(e))
  }

    /*

    Axios methods aren't working since the response type is json and cant be converted to blob

    const getImage =  () => {
        const formData = new FormData();
        formData.append('floor_id', '1');

        const options = {
          url: mapLink,
          method: 'POST',
          mode: 'cors',
          headers: {
            'Accept': 'application/json, text/plain, *//*',
          },
          data: formData
        }

        axios(options).
          then(response => {
              console.log(response)
              return response.blob()
          }).then(blob => {
              const link = URL.createObjectURL(blob)
              setImage(link)
          }).catch(e=>console.log(e))
    }
    */

    useEffect(() => { getImage(); }, []);

    return (
        <div className="map-portion">
            {/* <div style={{display: 'flex', justifyContent:'center'}}> */}
            {/* {image && <Image className="uwu" imageStyle={{width:"80%", height:"80%"}} src={image}/>} */}
            {image && <img src={image} className="map-image" alt="iwi"/>}
            {/* <svg className='exhibits' viewBox="0 0 200 250">
                    <polygon
                        style={{opacity: '20%', fill: 'green'}}
                        points="18,126 59,127 57,165 16,165"
                    />
                    <polygon
                        style={{opacity: '20%', fill: 'blue'}}
                        points="120,97 165,97.2 165,158 125,157"
                    />
            </svg> */}
            <PieceButton />
        </div>
    );
}

export default Map;