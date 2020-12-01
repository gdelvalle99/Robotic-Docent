import React, { useState, useEffect } from "react";
import Image from 'material-ui-image'

export const Map: React.FC = (props) => {

    const [image, setImage] = useState<string>("");

    const getImage = () => {
        const formData = new FormData();
        formData.append('floor_id', '1');

        fetch('http://localhost:5000/floor/update'
            , {
                method: 'POST',
                mode: 'cors',
                headers: {
                    'Accept': 'application/json, text/plain, */*',
                },
                body: formData
            }
        ).then(response => {
            return response.blob()
        }).then(blob => {
            const link = URL.createObjectURL(blob)
            setImage(link)
        }).catch(e=>console.log('uwu',e))
    }

    useEffect(() => { getImage(); }, []);

    return (
        <div className="map-portion">
            {image && <Image src={image}/>}
        </div>
    );
}