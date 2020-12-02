import React, { useState, useEffect } from "react";
import { ExpansionPanel, ExpansionPanelSummary, ExpansionPanelDetails } from '@material-ui/core';
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';

export interface Exhibit {
    id: number;
    floor_id: number;
    title: string;
    subtitle?: string;
    description?: string;
    start_date?: Date;
    end_date?: Date;
    theme?: string;
    questions?: string[];
    answers?: string[];
    pieces?: [{
        id: number;
        exhibit_id: number;
        title: string;
        author?: string;
        description?: string;
        origin?: string;
        era?: string;
        start_date?: Date;
        end_date?: Date;
        acquisition_date?: Date;
        dimension: number[];
        coordinates: number[];
        notes: string;
        questions?: string[];
        answers?: string[];
    }]
}

export const Editor: React.FC<Exhibit> = (props) => {

    const [exhibits, setExhibits] = useState<Exhibit[]>([]);

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