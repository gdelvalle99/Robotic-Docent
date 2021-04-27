import React, { useState, useEffect } from "react";
import { Map } from "../components/Map";
import { ExhibitSidebar } from '../components/ExhibitSidebar';
import { floorLink } from "../links";
import { exhibitLink } from "../links";
import axios from 'axios';

export const Editor = () => {
    const [exhibitList, setExhibitList] = useState([]);
    const [allPieces, setAllPieces] = useState([]);

    const getInitialExhibitList = () => {

        let r = axios.get(floorLink).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setExhibitList(e.exhibits);
            }).catch(e=>console.log(e))
    }

    const getInitialAllPieceList = () => {
        let iterateExhibits = exhibitList.map(exhibit => {
            let r = axios.get(exhibitLink+exhibit.id).
                then(function (response) {
                    return response;
                }).then(item => {
                    const e = item.data;
                    console.log(e);
                    setAllPieces(allPieces => [...allPieces, ...e.pieces])
                }).catch(e=>console.log(e)); 
        });
    }

    useEffect(() => { 
        getInitialExhibitList(); 
        getInitialAllPieceList();
    }, []);

    return (
        <div className="editor-page">
            <div className="editor-sidebar">
                <ExhibitSidebar exhibitList={exhibitList} setExhibitList={setExhibitList}/>
            </div>
            <div className="editor-map">
                <Map/>
            </div>
        </div>
    );
}
