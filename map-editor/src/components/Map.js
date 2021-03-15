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
import { mapLink } from '../links';
import Add from '@material-ui/icons/Add';
import axios from 'axios';

export const Map = (props) => {

    const [image, setImage] = useState("");
    const [open, setOpen] = useState(false);

    const handleClose = () => {
        setOpen(false);
    }

    const handleOpenClick = () => {
        setOpen(true);
    }
 
    const getImage = () => {
        const formData = new FormData();
        formData.append('floor_id', '476411ee-df58-4b98-8da6-4514d2fc0433'); // just for testing

        const token = localStorage.getItem('auth_token') || "";
        fetch(mapLink
            , {
                method: 'POST',
                headers: {
                    'Accept': 'application/json, text/plain, */*',
                    "Authentification": token,
                    "credentials": true
                },
                body: formData
            }
        ).then(response => {
            return response.blob()
        }).then(blob => {
            const link = URL.createObjectURL(blob)
            setImage(link)
        }).catch(e=>console.log(e))
  }

    /*

    Axios methods aren't working since the response type is json and cant be converted to blob

    const getImage =  () => {
        const formData = new FormData();
        formData.append('floor_id', '1');

        const options = {
          url: mapLink,
          method: 'POST',
          mode: 'cors',
          headers: {
            'Accept': 'application/json, text/plain, *//*',
          },
          data: formData
        }

        axios(options).
          then(response => {
              console.log(response)
              return response.blob()
          }).then(blob => {
              const link = URL.createObjectURL(blob)
              setImage(link)
          }).catch(e=>console.log(e))
    }
    */

    useEffect(() => { getImage(); }, []);

    return (
        <div className="map-portion">
            {/* <div style={{display: 'flex', justifyContent:'center'}}> */}
            {/* {image && <Image className="uwu" imageStyle={{width:"80%", height:"80%"}} src={image}/>} */}
            {image && <img src={image} className="map-image" alt="iwi"/>}
            {/* <svg className='exhibits' viewBox="0 0 200 250">
                    <polygon
                        style={{opacity: '20%', fill: 'green'}}
                        points="18,126 59,127 57,165 16,165"
                    />
                    <polygon
                        style={{opacity: '20%', fill: 'blue'}}
                        points="120,97 165,97.2 165,158 125,157"
                    />
            </svg> */}
            <Fab color="primary" aria-label="add" className="bottom-right" onClick={handleOpenClick}>
                <Add fontSize='large'/>
            </Fab>
            <Dialog open={open} onClose={handleClose} aria-labelledby="form-dialog-title">
        <DialogTitle id="form-dialog-title">Add a Museum Display</DialogTitle>
        <DialogContent>
        <InputLabel id="demo-simple-select-label">Exhibit Location</InputLabel>
        <Select
          labelId="demo-simple-select-label"
          id="demo-simple-select"
          style={{width: '100%'}}
        >
          <MenuItem value={10}>Modern Art</MenuItem>
          <MenuItem value={30}>Ancient Classical Art</MenuItem>
        </Select>
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Title"
            type="text"
            fullWidth
          />
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Author"
            type="text"
            fullWidth
          />
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Description"
            type="text"
            fullWidth
            multiline
            rows={3}
          />
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Origin"
            type="text"
            fullWidth
          />
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Era"
            type="text"
            fullWidth
          />
          <InputLabel shrink>Acquisition Date</InputLabel>
          <TextField
            autoFocus
            margin="dense"
            id="name"
            type="date"
            
            fullWidth
          />
          <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Dimensions"
            type="text"
            fullWidth
          />
          {/* <TextField
            autoFocus
            margin="dense"
            id="name"
            label="Coordinates"
            type="text"
            fullWidth
          /> */}
        </DialogContent>
        <DialogActions>
          <Button onClick={handleClose} color="primary">
            Cancel
          </Button>
          <Button onClick={handleClose} color="primary">
            Add
          </Button>
        </DialogActions>
      </Dialog>
        </div>
    );
}

export default Map;