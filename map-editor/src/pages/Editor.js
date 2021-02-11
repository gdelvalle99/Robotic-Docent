import React, { useState, useEffect } from "react";
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';

export const Editor = (props) => {

    const [exhibits, setExhibits] = useState([]);

    const getExhibits = async () => {
        let r = await fetch('testdata.json'
            , {
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                }
            }
        ).then(function (response) {
            return response.json();
        })
        setExhibits(r);
    }

    useEffect(() => { getExhibits(); }, []);

    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                <ExhibitSidebar exhibits={exhibits} />
            </div>
                    <Map/>
        </div>
    );
}