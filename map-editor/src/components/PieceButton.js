import React, { useState, useEffect } from "react";
import { Fab } from '@material-ui/core';
import { Button } from '@material-ui/core';
import { Dialog } from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { DialogContent } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core';
import { Select } from '@material-ui/core';
import { MenuItem } from '@material-ui/core';
import { InputLabel } from '@material-ui/core';
import { Input } from '@material-ui/core';
import { InputAdornment } from '@material-ui/core';
import { floorLink } from "../links";
import { newPieceLink } from "../links";
import axios from 'axios';
import Add from '@material-ui/icons/Add';

export const PieceButton = (props) => {
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);
    const [exhibits, setExhibits] = useState([]);
    const [exhibitName, setExhibitName] = useState("");
    const [newDimension, setNewDimension] = useState([]);
    const [newCoordinates, setNewCoordinates] = useState([]);
    const [newPiece, setNewPiece] = useState({});

    const handleBeforeSubmit = () => {
        const data = newPiece;
        data['coordinates'] = props.coordinates;
        let r = axios.post(newPieceLink, data).catch(e=>console.log(e));
    }

    //Handles alert for delete
    const handleOpenAlert = () => {
        if (newPiece.title != null && newPiece.title != "") {
            setOpenDeleteAlert(true);
        }
        else {
            handleCancel();
        }
    }

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    }

    const handleDeleteClick = () => {
        handleCancel();
        handleCloseAlert();
    }

    //Handle changes for text fields
    const handleDropdownChange = (event) => {
        setExhibitName(event.target.value);
    }

    const handleDimensionChange = (event) => {
        const newDim = newDimension;
        newDim[parseInt(event.target.name)] = event.target.value;
        setNewDimension(newDim);
        const dimNumified = [];
        dimNumified[0] = parseInt(newDim[0]);
        dimNumified[1] = parseInt(newDim[1]);
        dimNumified[2] = parseInt(newDim[2]);
        handleAllChanges(newPiece, {
            dimension: dimNumified
        })
    }

    const handleCoordinateChange = (event) => {
        const newCoords = newCoordinates;
        newCoords[parseInt(event.target.name)] = event.target.value;
        setNewCoordinates(newCoords);
        handleAllChanges(newPiece, {
            coordinates: newCoords
        })
    }

    const handleChange = (event) => {
        handleAllChanges(newPiece, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (piece, diff) => {
        setNewPiece({...piece, ...diff});
    }

    //Handle save, cancel, open, and close
    const handleSave = () => {
        handleBeforeSubmit();
        props.addToPlaces(newPiece);
        setNewPiece({});
        setExhibitName("");
        props.handleClose();
    }

    const handleCancel = () => {
        props.setLoc([]);
        setNewPiece({});
        setExhibitName("");
        props.handleClose();
    }

    //Get initial exhibit list
    const getExhibits = () => {
        let r = axios.get(floorLink).
            then(function (response) {
                return response;
            }).then(item => {
                const e = item.data;
                setExhibits(e.exhibits);
            }).catch(e=>console.log(e))

        //setNewPiece({...newPiece, coordinates: props.coordinates});
    }

    useEffect(() => { 
        getExhibits(); 
    }, []);
    
    return (
        <div className="map-piece-button">
            <Fab aria-label="add" className="floating-piece-button" onClick={() => {props.handleOpenClick(); getExhibits();}}>
                <Add fontSize='large'/>
            </Fab>
            <Dialog open={props.open} onClose={handleOpenAlert} aria-labelledby="form-dialog-title">
                <DialogTitle className="piece-modal-title-container">Add a New Piece</DialogTitle>
                <DialogContent>
                    <div className="piece-modal-content-container">
                        <TextField
                            label="Title"
                            type="text"
                            variant="outlined"
                            name="title" 
                            defaultValue={newPiece.title}
                            onBlur={handleChange} 
                            multiline
                            fullWidth
                            required
                        />
                        <TextField
                            label="Author"
                            type="text"
                            variant="outlined"
                            name="author" 
                            defaultValue={newPiece.author}
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Description"
                            type="text"
                            multiline
                            rows={5}
                            variant="outlined"
                            name="description" 
                            defaultValue={newPiece.description}
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <div className="piece-modal-content-coordinates-container">
                            <TextField
                                label="Coordinates"
                                placeholder="X"
                                name="0"
                                type="number"
                                defaultValue={Math.round(props.coordinates[0] * 100) / 100 || 0}
                                variant="outlined"
                                InputLabelProps={{shrink:true}}
                                InputProps={{
                                    startAdornment: <InputAdornment position="start">(</InputAdornment>
                                }}
                                onBlur={handleCoordinateChange}
                            />
                            <p className="piece-modal-content-coordinates-comma">,</p>
                            <TextField
                                placeholder="Y"
                                name="1"
                                type="number"
                                defaultValue={Math.round(props.coordinates[1] * 100) / 100 || 0}
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">)</InputAdornment>
                                }}
                                onBlur={handleCoordinateChange}
                            />
                            <Button onClick={props.handleLocation} variant="outlined">
                                Choose a spot
                            </Button>
                        </div>
                        <TextField 
                            select
                            variant="outlined"
                            label="Exhibit"
                            value={exhibitName}
                            name="exhibit_id"
                            onChange={handleDropdownChange}
                            onBlur={handleChange}
                            fullWidth
                            required
                        >
                            {exhibits.map((exhibit) => (
                                <MenuItem key={exhibit.id} value={exhibit.id}>
                                    {exhibit.title}
                                </MenuItem>
                            ))}
                        </TextField>
                        <TextField
                            label="Acquistion Date"
                            type="date"
                            variant="outlined"
                            name="acquisition_date" 
                            defaultValue={newPiece.acquisition_date}
                            onBlur={handleChange} 
                            InputLabelProps={{ shrink: true }}
                            fullWidth
                        />
                        <div className="piece-modal-content-dimension-container">
                            <TextField
                                label="Dimensions"
                                placeholder="Length"
                                name="0"
                                defaultValue={newDimension[0] || 0}
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                InputLabelProps={{shrink:true}}
                                onBlur={handleDimensionChange}
                            />
                            <p className="piece-modal-content-dimension-dash">:</p>
                            <TextField
                                placeholder="Width"
                                name="1"
                                defaultValue={newDimension[1] || 0}
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                onBlur={handleDimensionChange}
                            />
                            <p className="piece-modal-content-dimension-dash">:</p>
                            <TextField
                                placeholder="Height"
                                name="2"
                                defaultValue={newDimension[2] || 0}
                                type="number"
                                variant="outlined"
                                InputProps={{
                                    endAdornment: <InputAdornment position="end">in.</InputAdornment>
                                }}
                                onBlur={handleDimensionChange}
                            />
                        </div>
                        <TextField
                            label="Origin"
                            type="text"
                            variant="outlined"
                            name="origin" 
                            defaultValue={newPiece.origin}
                            onBlur={handleChange} 
                            fullWidth
                        />
                        <TextField
                            label="Era"
                            type="text"
                            variant="outlined"
                            name="era" 
                            defaultValue={newPiece.era}
                            onBlur={handleChange} 
                            fullWidth
                        />
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="piece-modal-buttons">
                        <div className="piece-modal-button-cancel">
                            <Button onClick={handleOpenAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                        <div className="piece-modal-button-add">
                            <Button onClick={handleSave} variant="outlined">
                                Add
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
             <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Quit editing?</DialogTitle>
                <DialogActions>
                    <div className="piece-modal-delete-alert-buttons">
                        <div className="piece-modal-delete-alert-button-delete">
                            <Button onClick={handleDeleteClick} variant="outlined">
                            Delete
                            </Button>
                        </div>
                        <div className="piece-modal-delete-alert-button-cancel">
                            <Button onClick={handleCloseAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
        </div>
    )
}

export default PieceButton;