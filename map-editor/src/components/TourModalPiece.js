import React, { useState, useEffect } from "react";
import { Card } from '@material-ui/core';
import { CardHeader } from '@material-ui/core'; 
import { Dialog } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { DialogContent } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core'; 
import { Divider } from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { Button } from '@material-ui/core';
import DeleteIcon from '@material-ui/icons/Delete';

const convertSimpleISODate = function(dateString) {
    if (dateString) {
        let d = new Date(dateString);

        return d.toISOString().substring(0,10);
    }
    else {
        return null;
    }
}

export const TourModalPiece = (props) => {
    const [tour, setTour] = useState(props.tour);
    const [openModal, setOpenModal] = useState(false);
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);
    const [titleFlag, setTitleFlag] = useState(false);
    const [newFlag, setNewFlag] = useState(true);

    //Handles alert for delete
    const handleOpenAlert = () => {
        if (tour.title != null && tour.title != "") {
            setOpenDeleteAlert(true);
        }
        else {
            props.handleDeleteTour(tour.id);
        }
    }

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    }

    const handleDeleteClick = () => {
        props.handleDeleteTour(tour.id);
        handleCloseAlert();
    }

    //Handles opening and closing of modal with the validation for the title field being required
    const handleCloseModal = () => {
        if (tour.title != null && tour.title != "") {
            if (newFlag) {
                const idPromise = props.handleSaveNewTour(tour);
                idPromise.then(d => {
                    setTour({...tour, id: d});
                });
                setNewFlag(false);
            }
            else {
                props.handleSaveExistingTour(tour);
            }
            setTitleFlag(false);
            setOpenModal(false);
        }
        else {
            setTitleFlag(true);
        }
    }

    const handleOpenModal = () => {
        setOpenModal(true);
    }

    //Handles updates for each field
    const handleChange = (event) => {
        handleAllChanges(tour, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (tour, diff) => {
        setTour({...tour, ...diff});
    }

    //Initial set for current tours
    const setInitialTour = () => {
        setTour(props.tour);        
    }

    //Initial render of existent tours and opens modal on instancing new tour
    // useEffect(() => {
    //     if (props.tour.title == null) {
    //         handleOpenModal();
    //         setNewFlag(true);
    //     }
    //     else {
    //         setNewFlag(false);
    //     }
    //     setInitialTour(); 
    // }, []);

    return (
        <div id={"tour-item-id-" + tour.id} className="tour-item-container">
            <Card className="tour-item-card" onClick={handleOpenModal}>
                <CardHeader 
                    title={tour.title}
                    titleTypographyProps={{variant: 'h5'}}
                    subheader={tour.subtitle} 
                    subheaderTypographyProps={{variant: 'body1'}}
                />
            </Card>
            {/*<Dialog open={openModal} onClose={handleCloseModal} fullWidth={true}>
                <DialogTitle>
                    <div className="tour-item-header">
                        <TextField
                            error={titleFlag}
                            required={true} 
                            className="tour-item-title"
                            placeholder="Title" 
                            variant="outlined" 
                            name="title" 
                            onBlur={handleChange} 
                            defaultValue={tour.title}
                            fullWidth
                        />
                        <TextField 
                            className="tour-item-subtitle"
                            placeholder="Subtitle" 
                            size={"small"}
                            variant="outlined" 
                            name="subtitle" 
                            onBlur={handleChange} 
                            defaultValue={tour.subtitle}
                            fullWidth
                        />
                        <Divider variant="middle" />
                    </div>
                </DialogTitle>
                <DialogContent>
                    <div className="tour-item-content-container">
                        <TextField 
                            className="tour-item-description"
                            label="Description" 
                            variant="outlined" 
                            multiline 
                            rows={5}
                            name="description" 
                            onBlur={handleChange} 
                            defaultValue={tour.description} 
                            fullWidth
                        />
                        <div className="tour-item-dates">
                            <TextField 
                                className="tour-item-startdate" 
                                label="Start Date" 
                                variant="outlined" 
                                type="date" 
                                name="start_date" 
                                onBlur={handleChange} 
                                defaultValue={convertSimpleISODate(tour.start_date)} 
                                InputLabelProps={{ shrink: true }}
                                fullWidth 
                            />
                            <TextField 
                                className="tour-item-enddate" 
                                label="End Date" 
                                variant="outlined" 
                                type="date" 
                                name="end_date" 
                                onBlur={handleChange} 
                                defaultValue={convertSimpleISODate(tour.end_date)} 
                                InputLabelProps={{ shrink: true }} 
                                fullWidth
                            />
                        </div>
                        <TextField 
                            className="tour-item-theme" 
                            label="Theme" 
                            variant="outlined"
                            name="theme" 
                            onBlur={handleChange} 
                            defaultValue={tour.theme} 
                            fullWidth 
                        />
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="tour-delete-button">
                        <Button onClick={handleOpenAlert} variant="outlined">
                            <DeleteIcon fontSize='small'/>
                        </Button>
                    </div>
                </DialogActions>
            </Dialog>
            <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Delete this tour?</DialogTitle>
                <DialogActions>
                    <div className="tour-delete-alert-buttons">
                        <div className="tour-delete-alert-button-delete">
                            <Button onClick={handleDeleteClick} variant="outlined">
                            Delete
                            </Button>
                        </div>
                        <div className="tour-delete-alert-button-cancel">
                            <Button onClick={handleCloseAlert} variant="outlined">
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog> */}
        </div>
    );
}