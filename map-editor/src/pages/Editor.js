import React, { useState, useEffect } from "react";
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';
import { exhibitLink } from "../links";
import axios from 'axios';

export const Editor = () => {

    const [exhibitListContainer, setExhibitListContainer] = useState([]);

    const getExhibitListContainer = () => {
        const values = {
            "floor_id": 1
        }

        let r = axios.post(exhibitLink, values).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setExhibitListContainer(e);
            }).catch(e=>console.log(e))

    }

    useEffect(() => { getExhibitListContainer(); }, []);

    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                <ExhibitSidebar exhibitListContainer={exhibitListContainer} />
            </div>
            <div className="editor-map">
                <Map/>
            </div>
        </div>
    );
}
