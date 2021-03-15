import React, { useState, useEffect } from "react";
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';
import { floorLink } from "../links";
import axios from 'axios';

export const Editor = () => {

    const [exhibitListContainer, setExhibitListContainer] = useState([]);

    const getExhibitListContainer = () => {

        let r = axios.get(floorLink).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                console.log(e);
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
