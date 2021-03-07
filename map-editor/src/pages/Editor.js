import React, { useState, useEffect } from "react";
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';
import { exhibitLink } from "../links";
import axios from 'axios';

export const Editor = () => {

    const [exhibitsList, setExhibitsList] = useState([]);

    const getExhibitsList = () => {
        const values = {
            "floor_id": 1
        }

        let r = axios.post(exhibitLink, values).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setExhibitsList(e);
            }).catch(e=>console.log(e))
    }

    useEffect(() => { getExhibitsList(); }, []);

    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                <ExhibitSidebar exhibitsList={exhibitsList} />
            </div>
            <div className="editor-map">
                <Map/>
            </div>
        </div>
    );
}
