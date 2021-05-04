const baseLink = "http://738e20ddd67c.ngrok.io"
//const baseLink = "http://127.0.0.1:5000"
const mapLink = baseLink+"/floor/map/get"
const tourLink = baseLink+"/tour/start"
const floor_id = "2fe2ff38-cd4d-45b7-a3d2-fc18270f71bc"
//const floor_id = "476411ee-df58-4b98-8da6-4514d2fc0433" 
const tour_id = "dbfee438-e5dd-4056-ae2d-4b7a876d351f"
const floorLink = baseLink+"/floor/exhibits?id="+floor_id
const newExhibitLink = baseLink+"/exhibit/new"
const existingExhibitLink = baseLink+"/exhibit/update"
const deleteExhibitLink = baseLink+"/exhibit/delete?id="
const exhibitLink = baseLink+"/exhibit/pieces?id="
const newPieceLink = baseLink+"/piece/new"
const deletePieceLink = baseLink+"/piece/delete?id="
const newTourLink = baseLink+"/tour/new"
const piecesLink = baseLink + "/floor/pieces";
const validateLink = baseLink + "/auth/validate";
const createTourLink = baseLink+"/tour/create/addFloor";
const demoLink = "http://062b17e2823c.ngrok.io/start_tour?tour_id="

export {
    baseLink,
    mapLink,
    tourLink,
    floorLink,
    floor_id,
    tour_id,
    newExhibitLink,
    existingExhibitLink,
    deleteExhibitLink,
    exhibitLink,
    newPieceLink,
    demoLink,
    deletePieceLink, 
    newTourLink,
    validateLink,
    piecesLink,
    createTourLink,
};
