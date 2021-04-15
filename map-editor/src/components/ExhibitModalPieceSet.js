import React, { useState, useEffect } from "react";
import { ExhibitModalPiece } from './ExhibitModalPiece';
import { List } from '@material-ui/core';
import { ListItem } from '@material-ui/core';
import { ListItemText } from '@material-ui/core';
import { exhibitLink } from '../links';
import axios from 'axios';

export const ExhibitModalPieceSet = (props) => {
    const [pieceList, setPieceList] = useState([]);
    const [open, setOpen] = useState(false);

    const handleOpen = () => {
        setOpen(true);
    }

    const handleClose = () => {
        setOpen(false);
    }

    const setInitialPieceList = () => {
        let r = axios.get(exhibitLink+props.exhibit_id).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setPieceList(e.pieces);
            }).catch(e=>console.log(e))  
    }

    useEffect(() => {
        setInitialPieceList(); 
    }, []);

    return (
        <div className="exhibit-item-piece-container">
            <div className="exhibit-item-piece-header">
                <h4>Pieces</h4>
            </div>
            <div className="exhibit-item-piece-list-container">
                <List>
                    {pieceList && (
                            pieceList.map((piece, index) => {
                                return (                         
                                    <div className="exhibit-piece-item-modal">
                                        <ListItem button onClick={handleOpen}>
                                            <ListItemText primary={piece.title} secondary={piece.author}/>
                                        </ListItem>
                                        <ExhibitModalPiece key={index} piece={piece} open={open} handleClose={handleClose}/>
                                    </div>
                                );
                            })
                        )
                    }
                </List>
            </div>
        </div>
    );
}